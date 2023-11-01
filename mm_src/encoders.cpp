/*
 * File: encoders.cpp
 * Project: mazerunner
 * File Created: Saturday, 27th March 2021 3:50:10 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Monday, 5th April 2021 3:05:30 pm
 * Modified By: Peter Harrison
 * -----
 * MIT License
 *
 * Copyright (c) 2021 Peter Harrison
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
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

#include "encoders.h"
#include"profile.h"
#include "config.h"
#include <Arduino.h>

/****************************************************************************/
/*   ENCODERS                                                               */
/****************************************************************************/

/*
   from ATMega328p datasheet section 12:
   The ATMega328p can generate interrupt as a result of changes of state on two of its pins:

   PD2 for INT0  - Arduino Digital Pin 2
   PD3 for INT1  - Arduino Digital Pin 3

   The INT0 and INT1 interrupts can be triggered by a falling or rising edge or a low level.
   This is set up as indicated in the specification for the External Interrupt Control Register A –
   EICRA.

   The External Interrupt 0 is activated by the external pin INT0 if the SREG I-flag and the
   corresponding interrupt mask are set. The level and edges on the external INT0 pin that activate
   the interrupt are defined as

   ISC01 ISC00  Description
     0     0    Low Level of INT0 generates interrupt
     0     1    Logical change of INT0 generates interrupt
     1     0    Falling Edge of INT0 generates interrupt
     1     1    Rising Edge of INT0 generates interrupt


   The External Interrupt 1 is activated by the external pin INT1 if the SREG I-flag and the
   corresponding interrupt mask are set. The level and edges on the external INT1 pin that activate
   the interrupt are defined in Table 12-1

   ISC11 ISC10  Description
     0     0    Low Level of INT1 generates interrupt
     0     1    Logical change of INT1 generates interrupt
     1     0    Falling Edge of INT1 generates interrupt
     1     1    Rising Edge of INT1 generates interrupt

   To enable these interrupts, bits must be set in the external interrupt mask register EIMSK

   EIMSK:INT0 (bit 0) enables the INT0 external interrupt
   EIMSK:INT1 (bit 1) enables the INT1 external interrupt

*/

const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const float DEG_PER_MM_DIFFERENCE = (180.0 / (2 * MOUSE_RADIUS * PI));
static volatile bool turn_motion=0;
static volatile float s_robot_position;
static volatile float s_robot_angle;

static float s_robot_fwd_increment = 0;
static float s_robot_rot_increment = 0;

volatile unsigned long encoder_left_counter;
volatile unsigned long encoder_right_counter;

static volatile int32_t s_left_total;
static volatile int32_t s_right_total;
static volatile float speed_robot_now;

void reset_encoders() {
  cli();
    encoder_left_counter = 0;
    encoder_right_counter = 0;
    s_robot_position = 0;
    s_robot_angle = 0;
    s_left_total = 0;
    s_right_total = 0;
  sei();
}
portMUX_TYPE timerMux=portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR  left_int() {
  portENTER_CRITICAL_ISR(&timerMux);  
  encoder_left_counter += 1;
  portEXIT_CRITICAL_ISR(&timerMux);
}
// INT1 will respond to the XOR-ed pulse train from the right encoder
// runs in constant time of around 3us per interrupt.
// would be faster with direct port access
void IRAM_ATTR  right_int() {
   portENTER_CRITICAL_ISR(&timerMux); 
  encoder_right_counter += 1;
   portEXIT_CRITICAL_ISR(&timerMux);
}
void setup_encoders() {
  // left

    // left
    pinMode(ENCODER_LEFT_B, INPUT);
    // configure the pin change
    attachInterrupt(ENCODER_LEFT_B,left_int,FALLING);
    // attachInterrupt(ENCODER_RIGHT_B,right_int,FALLING);
    // enable the interrupt
    encoder_left_counter = 0;
    // right
    pinMode(ENCODER_RIGHT_B, INPUT);
    // configure the pin change
    attachInterrupt(ENCODER_RIGHT_B,right_int,FALLING);
    encoder_right_counter = 0;

  reset_encoders();
}
void attachInt()
{
  attachInterrupt(ENCODER_LEFT_B,left_int,FALLING);
  attachInterrupt(ENCODER_RIGHT_B,right_int,FALLING);
}
void IRAM_ATTR onTimer(){
  update_encoders();
  forward.update();
  rotation.update();
  // g_cross_track_error = update_wall_sensors();
  // g_steering_adjustment = calculate_steering_adjustment(g_cross_track_error);
  // update_motor_controllers(0);
  // start_sensor_cycle();
}
hw_timer_t * My_timer = NULL;
void setup_systick() {
   My_timer = timerBegin(1, 800, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  int timer_time=round(float(100000*LOOP_INTERVAL));
  Serial.println("timer:"+String(timer_time));
  timerAlarmWrite(My_timer, timer_time, true); // 1/250*1000000
  timerAlarmEnable(My_timer); //Just Enable
}
void detachInt()
{
      detachInterrupt(ENCODER_LEFT_B);
      detachInterrupt(ENCODER_RIGHT_B);
}
void update_encoders() {

  // Make sure values don't change while being read. Be quick.
 cli();
    int left_delta = encoder_left_counter;
    int right_delta = encoder_right_counter;
    encoder_left_counter = 0;
    encoder_right_counter = 0;
  sei();
  s_left_total += left_delta;
  s_right_total += right_delta;
  float left_change = left_delta * MM_PER_COUNT_LEFT;
  float right_change = right_delta * MM_PER_COUNT_RIGHT;
  s_robot_fwd_increment = 0.5 * (right_change + left_change);
  s_robot_rot_increment = (right_change - left_change) * DEG_PER_MM_DIFFERENCE;
  speed_robot_now=s_robot_fwd_increment*LOOP_FREQUENCY;
  s_robot_position += s_robot_fwd_increment;
  s_robot_angle += s_robot_rot_increment; 
}
float robot_speed()
{
   float distance;
 cli(); distance = speed_robot_now; sei();
  return distance;
}
float robot_position() {
  float distance;
 cli(); distance = s_robot_position; sei();
  return distance;
}

float robot_fwd_increment() {
  float distance;
  cli();distance = s_robot_fwd_increment; sei();
  // distance = s_robot_fwd_increment;
  return distance;
}

float robot_rot_increment() {
  float distance;
  cli(); distance = s_robot_rot_increment; sei();
  return distance;
}

float robot_angle() {
  float angle;
  cli(); angle = s_robot_angle; sei();
  return angle;
}

uint32_t encoder_left_total() {
  return s_left_total;
};

uint32_t encoder_right_total() {
  return s_right_total;
};

/**
 * Measurements indicate that even at 1500mm/s thetotal load due to
 * the encoder interrupts is less than 3% of the available bandwidth.
 */

// INT0 will respond to the XOR-ed pulse train from the leftencoder
// runs in constant time of around 3us per interrupt.
// would be faster with direct port access

