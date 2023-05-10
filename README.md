# Kitronik-Pico-Robotics-Board-C

A class and sample code to use the Kitronik Robotics board for Raspberry Pi Pico. (www.kitronik.co.uk/5329)

This is the C version. 

To use edit the `main.c` file, then build the project, and copy the `pico-robotics.uf2` file onto the Pico.

The `pico-robotics.uf2` file in the base of this repo is a piece of test code.

## Building main.c
- Edit `main.c` to include your code
- Edit the `CMakeLists.txt` to change the pico-sdk folder location
    - include(/path/to/your/pico-sdk/pico_sdk_init.cmake)
- Create a new folder called `build`
- In your terminal navigate to the `build` folder
- Execute the command ```cmake -DPICO_BOARD=pico .. && make```
- Your project file `pico-robotics.uf2` will now be available in the `build` folder

## Import the library and construct an instance:
```
#include "KitronikPicoRobotics.h"
initPicoRobotics();
```
This will initialise the PCA to default values.

## Drive a motor:
```
motorOn(motor, direction, speed)
```
where:
* __motor__ => 1 to 4
* __direction__ => f or r
* __speed__ => 0 to 100

## Stop a motor:
```
motorOff(motor)
```
where:
* __motor__ => 1 to 4

## Drive a Servo:
```
servoWrite(servo, degrees)
```
where:
* __servo__ => 1 to 8
* __degrees__ => 0 to 180

## Drive a Stepper:
```
step(stepperMotor, direction, steps)
```
where:
* __stepperMotor__ => 1 or 2 (stepper 1 is DC motors 1 and 2, stepper 2 is DC motors 3 and 4)
* __direction__ => f or r
* __steps__ => how many steps to make

### To step an angle:
```
stepAngle(stepperMotor, direction, angle)
```
where
* __stepperMotor__ => 1 or 2 (stepper 1 is DC motors 1 and 2, stepper 2 is DC motors 3 and 4)
* __direction__ => f or r
* __angle__ => how many degrees to move
<br/>

# Troubleshooting
If the code is run without the Pico Robotics board connected, or if the board is not powered up it is likely that the I2C functions in the library will return one of the following errors:
- PICO_ERROR_GENERIC if the given address was not acknowledged, or there was no device present,
- Or PICO_ERROR_TIMEOUT if a has timeout occurred.

This is because the Pico tries to communicate with an I2C device which is not responding.
