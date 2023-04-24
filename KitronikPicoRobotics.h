/*
MIT License

Copyright (c) 2023 Kitronik Ltd 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef KITRONIK_PICO_ROBOTICS_H_
#define KITRONIK_PICO_ROBOTICS_H_

// Class variables - these should be the same for all instances of the class.
// If you wanted to write some code that stepped through
// the servos or motors then this is the Base and size to do that
#define I2C_BAUDRATE 100000
#define I2C_ADDRESS 108
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

#define SRV_REG_BASE 0x08
#define MOT_REG_BASE 0x28
#define REG_OFFSET 4

void i2c_write_byte_to_mem(uint8_t addr, uint8_t val);

void initPCA();

void adjustServos(uint8_t change);
void servoWrite(uint8_t servo, uint8_t degrees);

void motorOn(uint8_t motor, char direction, uint8_t speed);
void motorOff(uint8_t motor);

void step(uint8_t motor, char direction, uint8_t steps);
void stepAngle(uint8_t motor, char direction, uint8_t angle);

void initPicoRobotics();

#endif // KITRONIK_PICO_ROBOTICS_H_
