/*
 * File: mazerunner.ino
 * Project: mazerunner
 * File Created: Monday, 5th April 2021 8:38:15 am
 * Author: Peter Harrison
 * -----
 * Last Modified: Thursday, 8th April 2021 8:38:41 am
 * Modified By: Peter Harrison
 * -----
 * MIT License
 *
 * Copyright (c) 2021 Peter Harrison
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, i  ncluding without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include"config.h"
#include "motors.h"
#include"motion.h"
#include"reports.h"
#include"encoders.h"

void setup() {
  Serial.begin(BAUDRATE);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_FRONT, OUTPUT);
  pinMode(SPEAK, OUTPUT);
  // enable_sensors();
  setup_motors();
 
  disable_motor_controllers();
  pinMode(but1,INPUT_PULLUP);
  pinMode(but2,INPUT_PULLUP);
  setup_encoders();
  reset_drive_system();
  setup_systick();
  Serial.println();
}

void loop() {

  static unsigned long time_d=millis();
  static float vol_now=0;
  static bool done_=0;
  if (millis() - time_d > 2000 && !done_) {
      float speed_=robot_speed();
   
      int voltage_to_pwm=round((float)(vol_now*83.7838+1.297));
      // Serial.println(voltage_to_pwm);
      set_left_motor_pwm(voltage_to_pwm);
      set_right_motor_pwm(voltage_to_pwm);
        String str_send="V:"+String(vol_now)+"-SP:"+String(speed_);
       Serial.println(str_send);
     
      time_d = millis();
    }
    if(digitalRead(but1)==0)
    {
      delay(200);
      vol_now+=0.5;
      if(vol_now>=3.5){
        vol_now=0;
      }
    }
}