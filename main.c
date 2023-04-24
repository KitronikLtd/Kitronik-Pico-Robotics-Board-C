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

#include "pico/stdlib.h"

#include "KitronikPicoRobotics.h"

bool USING_STEPPER_MOTORS = false;

int main() {
    stdio_init_all();
    initPicoRobotics();

    char directions[2] = {'f', 'r'};

    while (true) {
        if (USING_STEPPER_MOTORS) {
            // Using stepper motors
            for (uint8_t d = 0; d < 2; d++) {
                for (int8_t i = 0; i < 200; i++) {
                    step(1, directions[d], 8);
                    step(2, directions[d], 8);
                }
            }
        } else {
            // Using normal motors
            for (uint8_t d = 0; d < 2; d++) {
                for (int8_t s = 0; s <= 100; s++) {
                    motorOn(1, directions[d], s);
                    motorOn(2, directions[d], s);
                    motorOn(3, directions[d], s);
                    motorOn(4, directions[d], s);
                    sleep_ms(100);
                }
                for (int8_t s = 100; s >= 0; s--) {
                    motorOn(1, directions[d], s);
                    motorOn(2, directions[d], s);
                    motorOn(3, directions[d], s);
                    motorOn(4, directions[d], s);
                    sleep_ms(100);
                }
                sleep_ms(1000);
            }
        }

        // Using servos
        for (int16_t d = 0; d <= 180; d++) {
            servoWrite(1, d);
            servoWrite(2, d);
            servoWrite(3, d);
            servoWrite(4, d);
            servoWrite(5, d);
            servoWrite(6, d);
            servoWrite(7, d);
            servoWrite(8, d);
            sleep_ms(10);
        }
        for (int16_t d = 180; d >= 0; d--) {
            servoWrite(1, d);
            servoWrite(2, d);
            servoWrite(3, d);
            servoWrite(4, d);
            servoWrite(5, d);
            servoWrite(6, d);
            servoWrite(7, d);
            servoWrite(8, d);
            sleep_ms(10);
        }
    }

    return 0;
}
