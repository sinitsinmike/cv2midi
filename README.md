<img width="369" height="213" alt="CV2MIDI board" src="https://github.com/user-attachments/assets/b73a1cb7-605d-4423-8427-64a7c1ac0055" />

![CV2MIDI board](https://github.com/user-attachments/assets/8c210190-3927-46ec-b412-952fb3cd6b8d)

<img width="371" height="233" alt="CV2MIDI 3d" src="https://github.com/user-attachments/assets/0bd54a72-9b3e-4412-a07e-407d29a13128" />

![CV2MIDI 3d](https://github.com/user-attachments/assets/05978e49-6fb1-46c0-a86e-5572b2d1deee)

<img width="450" height="278" alt="CV2MIDI 2d" src="https://github.com/user-attachments/assets/6818681b-1256-4342-84c0-ecc4606a2c2a" />

![CV2MIDI 2d](https://github.com/user-attachments/assets/ff352d57-e257-47fc-9edc-7052a1d182b6)

What is missing here is CV and Gate input overvoltage protection. Somehow I haven't rhought of that when designing these PCBs.
The following schematics has to be added to both inputs:
<img width="683" height="560" alt="Protect overvoltage" src="https://github.com/user-attachments/assets/f26d6798-38f7-45a2-9e90-799cc4f8a569" />

3 components have to be added to each input. The PCB has configuration jumpers, so it would be easy to fix this by using those as soldering pads, to freewire this minimal addition.
