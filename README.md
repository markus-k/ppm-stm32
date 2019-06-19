# PPM to USB joystick interface for STM32F1 "BluePill"
This is a simple project for interfacing the PPM output of most R/C transmitters to a PC using a cheap STM32F1 "BluePill" board. The interface represents itself as a generic USB joystick and can be used with any simulation or game.

## Connecting the R/C transmitter
You need a PPM output either on your R/C transmitter, most often in form of a trainer port, or on the receiver. Connect the PPM pin to PA0 on the STM32 board and connect both grounds.

## Building
You need a STLink-v2 in order to flash the STM32 board.

After cloning, initialize and update the submodules, then build libopencm3:
```
git submodule update --init
cd libopencm3/
make TARGETS="stm32/f1"
cd ..
```

Then build and flash the firmware:
```
make
make flash
```

## License
This project is licensed under the GNU Public License Version 3.
