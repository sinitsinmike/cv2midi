// File: CVtoMIDI_V8p4_NoGliss_ConfigOrPin.ino
// 0–5V (1V/oct). Двухточечная калибровка по кнопке (D9), надёжная AUTO-полярность, анти-boot-note.
// Новое: режим «без глиссандо» через #define и/или пин D10 (джампер на GND).

#include <Arduino.h>
#include <EEPROM.h>

// ---------- USER CONFIG ----------

// База 0V: 1=всегда строка; 0=сначала DIP, потом строка
#define ZERO_VOLT_PRIORITY_NOTE   0
#define ZERO_VOLT_NOTE            "C3"

// DIP-пины (НЕ используйте D1 — это MIDI TX)
const uint8_t SEL_PINS[]      = {2, 3, 4, 5, 6, 8};
const int8_t  SEL_PINS_OCTS[] = {0, 1, 2, 3, 4, 5}; // C0..C5
#define SEL_USE_PULLUP           1
#define SEL_ACTIVE_LEVEL_LOW     1
#define LIVE_DIP_ENABLE          1
const uint16_t DIP_SCAN_MS    = 50;
const uint16_t DIP_STABLE_MS  = 100;

// --- Кнопка калибровки ---
#define CAL_PIN                  9      // кнопка на GND
#define CAL_USE_PULLUP           1
#define CAL_ACTIVE_LOW           1
const uint16_t CAL_HOLD_MS      = 1500; // долгое нажатие для входа

// --- Переключатель «без глиссандо» ---
// Три режима:
//   FORCE_OFF  = 0 — всегда с глиссандо (игнорируем D10)
//   AUTO       = 1 — читаем D10 (D10→GND = без глисса; отпущен = с глиссом)
//   FORCE_ON   = 2 — всегда без глиссандо (игнорируем D10)
#define NO_GLISS_MODE_FORCE_OFF 0
#define NO_GLISS_MODE_AUTO      1
#define NO_GLISS_MODE_FORCE_ON  2
#ifndef NO_GLISS_MODE
#define NO_GLISS_MODE NO_GLISS_MODE_AUTO
#endif

#define NO_GLISS_PIN             10
#define NO_GLISS_USE_PULLUP      1
#define NO_GLISS_ACTIVE_LOW      1

// Gate: AUTO/HIGH/LOW
#define GATE_POLARITY            "AUTO"  // "AUTO" | "HIGH" | "LOW"
const bool     AUTO_DEFAULT_ACTIVE_HIGH = true;
#define GATE_USE_PULLUP          1
const uint16_t GATE_DEBOUNCE_MS = 5;
const uint16_t GATE_DETECT_MS   = 120;

// Анти-нота-на-старте
const uint16_t STARTUP_ARM_IDLE_MS      = 20;
const bool     REQUIRE_EDGE_FOR_FIRST   = true;

// Калибровка (опоры и тайминги)
#define CAL_NOTE1_STR            "C4"
#define CAL_NOTE2_STR            "C6"
const uint16_t HOLD_SETTLE_MS         = 150;
const uint16_t SAMPLE_MS              = 200;
const uint16_t STEP_TIMEOUT_MS        = 30000;
const uint16_t CAL_TOTAL_TIMEOUT_MS   = 60000;
const uint16_t IDLE_BEFORE_STEP_MS    = 300;

// Фильтрация/квантайзер
const uint8_t  OVERSAMPLE         = 8;
const uint8_t  STAB_SAMPLES       = 64;
const uint8_t  TRIM_KEEP_PERCENT  = 60;
const uint16_t STABILIZE_MS       = 8;
const float    HYST_CENTS         = 25.0f;  // WHY: защита от дрожи границ
const bool     LEGATO_TRACKING    = true;

// Пины
const uint8_t PIN_CV   = A0;
const uint8_t PIN_GATE = 7;
const uint8_t PIN_LED  = 13;

// MIDI
const uint8_t MIDI_CHANNEL   = 1;
const uint8_t MIDI_VELOCITY  = 127;

// ---------- STATE ----------
struct CvSample { float adc_avg; };

bool  in_calibration = false;

bool  gate_active_high_rt = true;
bool  gate_active_stable  = false;
bool  gate_active_prev    = false;
bool  gate_active_raw_last = false;
unsigned long gate_last_flip_ms = 0;

bool          note_armed = false;
unsigned long gate_inactive_since_ms = 0;

bool  note_on = false;
int   current_note = -1;
float up_thr = 0.0f, down_thr = 0.0f;
unsigned long last_scan_ms = 0;

// DIP live
unsigned long dip_last_scan_ms=0, dip_last_flip_ms=0;
int  dip_raw_midi = -2, dip_stable_midi = -1, pending_zero_midi = -1;

// карта ADC→полутон
int   ZERO_VOLT_MIDI = 60;
float g_a = 60.0f / 1023.0f;
float g_b = 0.0f;
float g_adc_zero = 0.0f;

// ---------- UTILS ----------
static inline int clampMidi(int n){ if(n<0) return 0; if(n>127) return 127; return n; }
static inline int fastRoundToInt(float x){ return (int)(x >= 0 ? x + 0.5f : x - 0.5f); }
int parseNoteName(const char* s){
  if(!s || !s[0]) return 60;
  char L=s[0]; if(L>='a'&&L<='g') L = (char)(L - 'a' + 'A');
  int base=(L=='C')?0:(L=='D')?2:(L=='E')?4:(L=='F')?5:(L=='G')?7:(L=='A')?9:(L=='B')?11:-1;
  if(base<0) return 60;
  int i=1, acc=0; if(s[i]=='#'){acc=+1;i++;} else if(s[i]=='b'||s[i]=='B'){acc=-1;i++;}
  bool neg=false; if(s[i]=='-'){neg=true;i++;}
  int oct=0; bool has=false; while(s[i]>='0'&&s[i]<='9'){has=true; oct=oct*10+(s[i]-'0'); i++;}
  if(!has) return 60; if(neg) oct=-oct;
  long midi=(long)(oct+1)*12L+base+acc; if(midi<0)midi=0; if(midi>127)midi=127; return (int)midi;
}

// ---------- EEPROM CAL ----------
struct CalBlob { uint32_t magic; float a; float b; uint8_t crc; };
const uint32_t CAL_MAGIC = 0xC015FACE;
const int      CAL_ADDR  = 0;
uint8_t crc8(const uint8_t* p,size_t n){ uint8_t c=0; for(size_t i=0;i<n;++i) c^=p[i]; return c; }
void saveCalibration(float a,float b){
  CalBlob cb{CAL_MAGIC,a,b,0}; cb.crc = crc8((uint8_t*)&cb,sizeof(CalBlob)-1); EEPROM.put(CAL_ADDR,cb);
}
bool loadCalibration(float& a,float& b){
  CalBlob cb; EEPROM.get(CAL_ADDR,cb);
  if(cb.magic!=CAL_MAGIC) return false;
  uint8_t c=crc8((uint8_t*)&cb,sizeof(CalBlob)-1); if(c!=cb.crc) return false;
  if(!(cb.a>0.00001f)) return false; a=cb.a; b=cb.b; return true;
}

// ---------- MIDI ----------
static inline uint8_t midiStatusOn(uint8_t ch){ return 0x90 | ((ch-1)&0x0F); }
static inline uint8_t midiStatusOff(uint8_t ch){ return 0x80 | ((ch-1)&0x0F); }
void midiBegin(){ Serial.begin(31250); }
void midiWrite(uint8_t b){ Serial.write(b); }
void midiNoteOn(uint8_t note,uint8_t vel){ midiWrite(midiStatusOn(MIDI_CHANNEL)); midiWrite(note); midiWrite(vel); }
void midiNoteOff(uint8_t note){ midiWrite(midiStatusOff(MIDI_CHANNEL)); midiWrite(note); midiWrite((uint8_t)0); }

// ---------- ADC ----------
CvSample readAdcOversampled(){ long sum=0; (void)analogRead(PIN_CV);
  for(uint8_t i=0;i<OVERSAMPLE;++i) sum += analogRead(PIN_CV);
  return CvSample{ (float)sum/(float)OVERSAMPLE };
}
CvSample readAdcTrimmed(uint8_t n,uint8_t keep,uint16_t dur_ms){
  if(n<8)n=8; if(n>64)n=64; float buf[64]; const unsigned long t_end=millis()+dur_ms;
  for(uint8_t i=0;i<n;++i){ buf[i]=analogRead(PIN_CV); if(dur_ms && millis()>=t_end) break; }
  for(uint8_t i=1;i<n;++i){ float k=buf[i]; int j=i-1; while(j>=0&&buf[j]>k){buf[j+1]=buf[j];--j;} buf[j+1]=k; }
  uint8_t keepn=(uint8_t)((uint16_t)n*keep/100); if(keepn<1)keepn=1; if(keepn>n)n=n;
  uint8_t drop=(n>keepn)?(uint8_t)((n-keepn)/2):0; float sum=0.0f; for(uint8_t i=0;i<keepn;++i) sum+=buf[drop+i];
  return CvSample{ sum/keepn };
}

// ---------- Schmitt ----------
static inline void updateSchmittThresholds(int note){
  const float h=HYST_CENTS/100.0f; up_thr=(float)note+0.5f+h; down_thr=(float)note-0.5f-h;
}

// ---------- Gate ----------
inline bool readGateRawHigh(){ return digitalRead(PIN_GATE)==HIGH; }
void detectGatePolarityEnhanced(){
  if (GATE_POLARITY[0]=='H'){ gate_active_high_rt=true;  return; }
  if (GATE_POLARITY[0]=='L'){ gate_active_high_rt=false; return; }
  bool startHigh = readGateRawHigh();
  uint16_t toggles=0, highs=0, lows=0; bool prev=startHigh;
  const unsigned long t0=millis();
  while(millis()-t0 < GATE_DETECT_MS){
    bool h=readGateRawHigh(); if(h) highs++; else lows++; if(h!=prev){toggles++; prev=h;} delay(1);
  }
  if(toggles==0){ gate_active_high_rt = AUTO_DEFAULT_ACTIVE_HIGH; }
  else{ bool idleIsHigh = highs >= lows; gate_active_high_rt = !idleIsHigh; }
}
inline bool gateRawActiveByPolarity(){ const bool h=readGateRawHigh(); return gate_active_high_rt ? h : !h; }
void updateGateStable(){
  const unsigned long now=millis();
  const bool rawActive=gateRawActiveByPolarity();
  if(rawActive!=gate_active_raw_last){ gate_active_raw_last=rawActive; gate_last_flip_ms=now; }
  if(now - gate_last_flip_ms >= GATE_DEBOUNCE_MS){ gate_active_stable=rawActive; }
  if(!note_armed){
    if(!gate_active_stable){
      if(gate_inactive_since_ms==0) gate_inactive_since_ms = now;
      if(now - gate_inactive_since_ms >= STARTUP_ARM_IDLE_MS) note_armed = true;
    }else{
      gate_inactive_since_ms = 0;
    }
  }
}

// ---------- CAL кнопка ----------
inline bool calRawActive(){
  int r = digitalRead(CAL_PIN);
  return CAL_ACTIVE_LOW ? (r==LOW) : (r==HIGH);
}
bool tryEnterCalibrationByButton(){
  static bool armed = true;
  static bool prev  = false;
  static unsigned long tDown = 0;
  bool cur = calRawActive();
  if(cur && !prev){ tDown = millis(); }
  if(!cur){ armed = true; }
  if(cur && armed && (millis() - tDown >= CAL_HOLD_MS)){ armed = false; return true; }
  prev = cur; return false;
}

// ---------- NO-GLISS логика ----------
inline bool noGlissPinActive(){
  int r = digitalRead(NO_GLISS_PIN);
  return NO_GLISS_ACTIVE_LOW ? (r==LOW) : (r==HIGH);
}
inline bool noGlissEffective(){
#if NO_GLISS_MODE == NO_GLISS_MODE_FORCE_ON
  return true;
#elif NO_GLISS_MODE == NO_GLISS_MODE_FORCE_OFF
  return false;
#else
  return noGlissPinActive();
#endif
}

// ---------- DIP ----------
void setupDipPins(){
#if !ZERO_VOLT_PRIORITY_NOTE
  const size_t N=sizeof(SEL_PINS)/sizeof(SEL_PINS[0]);
  for(size_t i=0;i<N;++i){
  #if SEL_USE_PULLUP
    pinMode(SEL_PINS[i], INPUT_PULLUP);
  #else
    pinMode(SEL_PINS[i], INPUT);
  #endif
  }
#endif
}
int pinsSelectZeroVoltMidi(){
#if ZERO_VOLT_PRIORITY_NOTE
  return -1;
#else
  const size_t N=sizeof(SEL_PINS)/sizeof(SEL_PINS[0]);
  for(size_t i=0;i<N;++i){
    int v=digitalRead(SEL_PINS[i]);
    bool active = SEL_ACTIVE_LEVEL_LOW ? (v==LOW) : (v==HIGH);
    if(active){ int oct=SEL_PINS_OCTS[i]; return clampMidi((oct+1)*12); }
  }
  return -1;
#endif
}
inline void recomputeAdcZeroFromBase(){ g_adc_zero = (g_a>0.000001f) ? ((float)ZERO_VOLT_MIDI - g_b)/g_a : 0.0f; }
void configureZeroVoltInitial(){
#if ZERO_VOLT_PRIORITY_NOTE
  ZERO_VOLT_MIDI = parseNoteName(ZERO_VOLT_NOTE);
#else
  setupDipPins(); int sel=pinsSelectZeroVoltMidi();
  ZERO_VOLT_MIDI = (sel>=0) ? sel : parseNoteName(ZERO_VOLT_NOTE);
#endif
  recomputeAdcZeroFromBase();
}
void liveDipScan(){
#if LIVE_DIP_ENABLE && !ZERO_VOLT_PRIORITY_NOTE
  const unsigned long now=millis();
  if(now - dip_last_scan_ms < DIP_SCAN_MS) return;
  dip_last_scan_ms=now;
  int raw=pinsSelectZeroVoltMidi(); if(raw!=dip_raw_midi){ dip_raw_midi=raw; dip_last_flip_ms=now; }
  if(now - dip_last_flip_ms >= DIP_STABLE_MS){
    if(raw!=dip_stable_midi){
      dip_stable_midi=raw; int target=(raw>=0)?raw:parseNoteName(ZERO_VOLT_NOTE);
      if(target!=ZERO_VOLT_MIDI){
        if(!gate_active_stable){ ZERO_VOLT_MIDI=target; recomputeAdcZeroFromBase(); pending_zero_midi=-1; }
        else                   { pending_zero_midi=target; }
      }
    }
  }
#endif
}

// ---------- LED helpers ----------
void ledBlink(uint8_t n,uint16_t on_ms,uint16_t off_ms){
  for(uint8_t i=0;i<n;++i){ digitalWrite(PIN_LED,HIGH); delay(on_ms); digitalWrite(PIN_LED,LOW); delay(off_ms); }
}
void ledPattern_waitNote1(){ digitalWrite(PIN_LED,HIGH); delay(100); digitalWrite(PIN_LED,LOW); delay(500); }
void ledPattern_waitNote2(){ for(uint8_t i=0;i<2;++i){ digitalWrite(PIN_LED,HIGH); delay(80); digitalWrite(PIN_LED,LOW); delay(100);} delay(440); }
void ledConfirmShot(){ digitalWrite(PIN_LED,HIGH); delay(250); digitalWrite(PIN_LED,LOW); delay(150); }
void ledNeedRelease(){ digitalWrite(PIN_LED,HIGH); delay(40); digitalWrite(PIN_LED,LOW); delay(160); }

// ---------- помощники мастера ----------
bool waitInactiveStable(uint16_t stable_ms,uint16_t timeout_ms, bool blink_release){
  const unsigned long t0=millis(); unsigned long startLow=0;
  while(millis()-t0 < timeout_ms){
    updateGateStable();
    if(!gate_active_stable){
      if(startLow==0) startLow=millis();
      if(millis()-startLow >= stable_ms) return true;
    }else{
      startLow=0; if(blink_release) ledNeedRelease();
    }
  }
  return false;
}
bool waitActiveEdge(uint16_t timeout_ms){
  const unsigned long t0=millis(); bool prev=gate_active_stable;
  while(millis()-t0 < timeout_ms){
    updateGateStable();
    if(!prev && gate_active_stable) return true;
    prev = gate_active_stable; delay(2);
  }
  return false;
}
bool capturePointEdge(float& out_adc, uint16_t total_timeout_ms, bool is_step2){
  const unsigned long t0=millis();
  if(!waitInactiveStable(IDLE_BEFORE_STEP_MS, total_timeout_ms, true)) return false;
  while(true){
    if(millis()-t0 > total_timeout_ms) return false;
    if(waitActiveEdge(10)) break;
    if(is_step2) ledPattern_waitNote2(); else ledPattern_waitNote1();
  }
  delay(HOLD_SETTLE_MS);
  out_adc = readAdcTrimmed(STAB_SAMPLES, TRIM_KEEP_PERCENT, SAMPLE_MS).adc_avg;
  ledConfirmShot();
  if(!waitInactiveStable(100, total_timeout_ms, false)) return false;
  return true;
}

// ---------- калибровка ----------
bool calibrationWizard(){
  in_calibration = true;  // WHY: глушим MIDI/трекинг
  ledBlink(2,120,120);
  const int n1=parseNoteName(CAL_NOTE1_STR), n2=parseNoteName(CAL_NOTE2_STR);
  const unsigned long T0=millis();
  float adc1=0, adc2=0;
  if(!capturePointEdge(adc1, STEP_TIMEOUT_MS, false)) { in_calibration=false; return false; }
  if(millis()-T0 > CAL_TOTAL_TIMEOUT_MS)              { in_calibration=false; return false; }
  if(!capturePointEdge(adc2, STEP_TIMEOUT_MS, true))  { in_calibration=false; return false; }
  float da = adc2 - adc1; if(da<0) da = -da; if(da < 1.0f) { in_calibration=false; return false; }
  float a = ((float)(n2 - n1)) / (adc2 - adc1);
  float b = (float)n1 - a*adc1;
  saveCalibration(a,b); g_a=a; g_b=b; recomputeAdcZeroFromBase();
  ledBlink(3,250,250);
  in_calibration = false;
  return true;
}

// ---------- Arduino ----------
void setup(){
  midiBegin();

#if GATE_USE_PULLUP
  pinMode(PIN_GATE, INPUT_PULLUP);
#else
  pinMode(PIN_GATE, INPUT);
#endif
  pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, LOW);

#if CAL_USE_PULLUP
  pinMode(CAL_PIN, INPUT_PULLUP);
#else
  pinMode(CAL_PIN, INPUT);
#endif

#if NO_GLISS_MODE == NO_GLISS_MODE_AUTO
  #if NO_GLISS_USE_PULLUP
    pinMode(NO_GLISS_PIN, INPUT_PULLUP);
  #else
    pinMode(NO_GLISS_PIN, INPUT);
  #endif
#endif

  detectGatePolarityEnhanced();

  for(uint8_t i=0;i<10;i++){ updateGateStable(); delay(1); }
  gate_active_prev = gate_active_stable;
  note_armed = false; gate_inactive_since_ms = 0;

  float la,lb; if(loadCalibration(la,lb)){ g_a=la; g_b=lb; }
  configureZeroVoltInitial();
}

void loop(){
  if(!in_calibration && tryEnterCalibrationByButton()) calibrationWizard();
  if(in_calibration){ delay(1); return; }

  liveDipScan();
  updateGateStable();

  if(!gate_active_stable && pending_zero_midi>=0){
    ZERO_VOLT_MIDI = pending_zero_midi; recomputeAdcZeroFromBase(); pending_zero_midi=-1;
  }

  if(REQUIRE_EDGE_FOR_FIRST){
    if(note_armed && !gate_active_prev && gate_active_stable){
      CvSample s=readAdcTrimmed(STAB_SAMPLES,TRIM_KEEP_PERCENT,STABILIZE_MS);
      float semi_abs = (float)ZERO_VOLT_MIDI + g_a * (s.adc_avg - g_adc_zero);
      current_note = clampMidi(fastRoundToInt(semi_abs));
      updateSchmittThresholds(current_note);
      midiNoteOn(current_note, MIDI_VELOCITY);
      note_on=true; digitalWrite(PIN_LED,HIGH);
    }
  }else{
    if(!gate_active_prev && gate_active_stable){
      CvSample s=readAdcTrimmed(STAB_SAMPLES,TRIM_KEEP_PERCENT,STABILIZE_MS);
      float semi_abs = (float)ZERO_VOLT_MIDI + g_a * (s.adc_avg - g_adc_zero);
      current_note = clampMidi(fastRoundToInt(semi_abs));
      updateSchmittThresholds(current_note);
      midiNoteOn(current_note, MIDI_VELOCITY);
      note_on=true; digitalWrite(PIN_LED,HIGH);
    }
  }

  if(gate_active_prev && !gate_active_stable){
    if(note_on){ midiNoteOff(current_note); note_on=false; digitalWrite(PIN_LED,LOW); }
  }

  if(gate_active_stable && note_on && LEGATO_TRACKING){
    const unsigned long now=millis();
    if(now - last_scan_ms >= 5){
      last_scan_ms=now;
      float semi_abs = (float)ZERO_VOLT_MIDI + g_a * (readAdcOversampled().adc_avg - g_adc_zero);

      if(noGlissEffective()){
        if(semi_abs >= up_thr || semi_abs <= down_thr){
          int next = clampMidi(fastRoundToInt(semi_abs));
          if(next != current_note){
            midiNoteOff(current_note);
            current_note = next;
            updateSchmittThresholds(current_note);
            midiNoteOn(current_note, MIDI_VELOCITY);
          }
        }
      }else{
        int next=current_note;
        if(semi_abs >= up_thr && current_note<127) next=current_note+1;
        else if(semi_abs <= down_thr && current_note>0) next=current_note-1;
        if(next!=current_note){
          midiNoteOff(current_note);
          current_note = next;
          updateSchmittThresholds(current_note);
          midiNoteOn(current_note, MIDI_VELOCITY);
        }
      }
    }
  }

  gate_active_prev = gate_active_stable;
}