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

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "KitronikPicoRobotics.h"

/*
*
Library for the Kitronik Robotics Board for Raspberry Pi Pico.
*
*/

uint8_t PRESCALE_VAL = 0x79;

/* Quick helper function for single byte transfers */
void i2c_write_byte_to_mem(uint8_t addr, uint8_t val) {
    uint8_t src[2] = {addr, val};
    i2c_write_blocking(i2c0, I2C_ADDRESS, src, 2, false);
}

// setup the PCA chip for 50Hz and zero out registers.
void initPCA() {
    // Make sure we are in a known position
    // Soft reset of the I2C chip
    uint8_t val = 0x06;
    i2c_write_blocking(i2c0, I2C_ADDRESS, &val, 1, false);

    // setup the prescale to have 20mS pulse repetition - this is dictated by the servos.
    // set PWM Frequency Pre Scale.  The prescale value is determined with the formunla:
    // presscale value = round(osc clock / (4096 * update rate))
    // Where update rate is the output modulation frequency required.
    // For example, the output frequency of 50Hz (20ms) for the servo, with the internal oscillator 
    // clock frequency of 25 Mhz is as follows:
    // prescale value = round( 25MHZ / (4096 * 50Hz) ) - 1 
    // prescale value = round (25000000 / (4096 * 50)) - 1 
    // presscale value = 121 = 79h = 0x79
    i2c_write_byte_to_mem(0xfe, PRESCALE_VAL);

    // block write outputs to off
    i2c_write_byte_to_mem(0xfa, 0x00);
    i2c_write_byte_to_mem(0xfb, 0x00);
    i2c_write_byte_to_mem(0xfc, 0x00);
    i2c_write_byte_to_mem(0xfd, 0x00);
    
    // come out of sleep
    i2c_write_byte_to_mem(0x00, 0x01);
    
    // It takes 500uS max for the oscillator to be up and running once the SLEEP bit (bit 4) has
    // been set to logic 0.  Timings on outputs are not guranteed if the PWM control registers are
    // accessed within the 500uS window.
    sleep_us(500);
}

// Adjusts the servos.
// This block should be used if the connected servo does not respond correctly to the 'servoWrite' command.
// Try changing the value by small amounts and testing the servo until it correctly sets to the angle.
void adjustServos(uint8_t change) {
    if (change < -25) {
        change = -25;
    }
    if (change > 25) {
        change = 25;
    }
    PRESCALE_VAL = 0x79 + change;
    initPCA();
}

// To get the PWM pulses to the correct size and zero
// offset these are the default numbers.
// Servo multiplier is calcualted as follows:
// 4096 pulses ->20mS 1mS-> count of 204.8
// 1mS is 90 degrees of travel, so each degree is a count of 204.8/90->2.2755
// servo pulses always have  aminimum value - so there is guarentees to be a pulse.
// in the servos Ive examined this is 0.5ms or a count of 102
// to clauclate the count for the corect pulse is simply:
// (degrees x count per degree )+ offset 
void servoWrite(uint8_t servo, uint8_t degrees) {
    // check the degrees is a reasonable number. we expect 0-180, so cap at those values.
    if (degrees > 180) {
        degrees = 180;
    } else if (degrees < 0) {
        degrees = 0;
    }
    
    if (servo < 1) {
        servo = 1;
    }
    if (servo > 8) {
        servo = 8;
    }

    uint8_t calcServo = SRV_REG_BASE + ((servo - 1) * REG_OFFSET);
    uint16_t PWMVal = (uint16_t) (degrees * 2.2755) + 102; // see comment above for maths
    uint8_t lowByte = PWMVal & 0xFF;
    uint8_t highByte = (PWMVal >> 8) & 0x01; // cap high byte at 1 - shoud never be more than 2.5mS.
    i2c_write_byte_to_mem(calcServo, lowByte);
    i2c_write_byte_to_mem(calcServo + 1, highByte);
}

// Driving the motor is simpler than the servo - just convert 0-100% to 0-4095 and push it to the correct registers.
// each motor has 4 writes - low and high bytes for a pair of registers. 
void motorOn(uint8_t motor, char direction, uint8_t speed) {
    // cap speed to 0-100%
    if (speed < 0) {
        speed = 0;
    } else if (speed > 100) {
        speed = 100;
    }

    if (motor < 1) {
        motor = 1;
    }
    if (motor > 4) {
        motor = 4;
    }
    
    uint8_t motorReg = MOT_REG_BASE + (2 * (motor - 1) * REG_OFFSET);
    uint16_t PWMVal = (uint16_t) (speed * 40.95);
    uint8_t lowByte = PWMVal & 0xFF;
    uint8_t highByte = (PWMVal >> 8) & 0xFF; // motors can use all 0-4096
    
    if (direction == 'f') {
        i2c_write_byte_to_mem(motorReg, lowByte);
        i2c_write_byte_to_mem(motorReg + 1, highByte);
        i2c_write_byte_to_mem(motorReg + 4, 0);
        i2c_write_byte_to_mem(motorReg + 5, 0);
    } else {
        i2c_write_byte_to_mem(motorReg, 0);
        i2c_write_byte_to_mem(motorReg + 1, 0);
        i2c_write_byte_to_mem(motorReg + 4, lowByte);
        i2c_write_byte_to_mem(motorReg + 5, highByte);
    }
}

// To turn off set the speed to 0...
void motorOff(uint8_t motor) {
    motorOn(motor, 'f', 0);
}

// // // // // // // // // // // // // // // // // 
// Stepper Motors
// // // // // // // // // // // // // // // // // 
// this is only a basic full stepping.
// speed sets the length of the pulses (and hence the speed...)
// so is 'backwards' - the fastest that works reliably with the motors I have to hand is 20mS, but slower than that is good. tested to 2000 (2 seconds per step).
// motor should be 1 or 2 - 1 is terminals for motor 1 and 2 on PCB, 2 is terminals for motor 3 and 4 on PCB
void step(uint8_t motor, char direction, uint8_t steps) {
    uint8_t speed = 20;
    bool holdPosition = false;

    if (motor < 1) {
        motor = 1;
    }
    if (motor > 2) {
        motor = 2;
    }

    char directions[2];
    uint8_t coils[2];
    if (direction == 'f') {
        directions[0] = 'f';
        directions[1] = 'r';
        coils[0] = (motor * 2) - 1;
        coils[1] = motor * 2;
    } else {
        directions[0] = 'r';
        directions[1] = 'f';
        coils[0] = motor * 2;
        coils[1] = (motor * 2) - 1;
    }

    while (steps > 0) {
        for (uint8_t d = 0; d < 2; d++) {
            if (steps == 0) {
                break;
            }

            for (uint8_t c = 0; c < 2; c++) {
                motorOn(coils[c], directions[d], 100);
                sleep_ms(speed);
                steps--;
                if (steps == 0) {
                    break;
                }
            }
        }
    }

    // to save power turn off the coils once we have finished.
    // this means the motor wont hold position.
    if (!holdPosition) {
        for (uint8_t c = 0; c < 2; c++) {
            motorOff(coils[c]);
        }
    }
}

// Step an angle. this is limited by the step resolution - so 200 steps is 1.8 degrees per step for instance.
// a request for 20 degrees with 200 steps/rev will result in 11 steps - or 19.8 rather than 20.
void stepAngle(uint8_t motor, char direction, uint8_t angle) {
    uint8_t stepsPerRev = 200;
    uint8_t steps = (uint8_t) angle / (360 / stepsPerRev);
    step(motor, direction, steps);
}

// Initialise all of the Kitronik Robotics Board components.
void initPicoRobotics() {
    // Initialise I2C data pin
    gpio_init(I2C_SDA_PIN);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    // Initialise I2C clock pin
    gpio_init(I2C_SCL_PIN);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL_PIN);
    // Use I2C0 on the SDA pin 8 and SCL pin 9
    i2c_init(i2c0, I2C_BAUDRATE);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));

    initPCA();
}
