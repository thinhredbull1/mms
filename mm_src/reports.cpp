/*
 * File: logger.cpp
 * Project: mazerunner
 * File Created: Tuesday, 23rd March 2021 10:18:00 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Thursday, 29th April 2021 9:44:01 am
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

#include "reports.h"
#include "encoders.h"
#include "motors.h"
#include "profile.h"
#include "sensors.h"
#include <Arduino.h>

static uint32_t start_time;
static uint32_t report_time;
static uint32_t report_interval = REPORTING_INTERVAL;

// note that the Serial device has a 64 character buffer and, at 115200 baud
// 64 characters will take about 6ms to go out over the wire.
void report_profile_header() {
#if DEBUG_LOGGING == 1
  Serial.println(F("time robotPos robotAngle fwdPos  fwdSpeed rotpos rotSpeed fwdVolts rotVolts"));
  start_time = millis();
  report_time = start_time;
#endif
}

void report_profile() {
#if DEBUG_LOGGING == 1
  if (millis() >= report_time) {
    report_time += report_interval;
    Serial.print(millis() - start_time);
    Serial.print(' ');
    Serial.print(robot_position());
    Serial.print(' ');
    Serial.print(robot_angle());
    Serial.print(' ');
    Serial.print(forward.position());
    Serial.print(' ');
    Serial.print(forward.speed());
    Serial.print(' ');
    Serial.print(rotation.position());
    Serial.print(' ');
    Serial.print(rotation.speed());
    Serial.print(' ');
    Serial.print(50 * (g_right_motor_volts + g_left_motor_volts));
    Serial.print(' ');
    Serial.print(50 * (g_right_motor_volts - g_left_motor_volts));
    Serial.println();
  }
#else
  delay(2);
#endif
}

//***************************************************************************//

void report_sensor_calibration() {
  Serial.println(F("left left_ref front front_ref right right_ref"));
  enable_sensors();
  start_time = millis();
  report_time = start_time;
  while (not button_pressed()) {
    if (millis() >= report_time) {
      report_time += 100;
      report_wall_sensors();
    }
  }
  Serial.println();
  wait_for_button_release();
  delay(200);
  disable_sensors();
}
//***************************************************************************//

void report_sensor_track_header() {
#if DEBUG_LOGGING == 1
  Serial.println(F("time pos angle left right front error adjustment"));
  start_time = millis();
  report_time = start_time;
#endif
}

void report_sensor_track() {
#if DEBUG_LOGGING == 1
  if (millis() >= report_time) {
    report_time += report_interval;
    Serial.print(millis() - start_time);
    Serial.print(' ');
    Serial.print(robot_position());
    Serial.print(' ');
    Serial.print(robot_angle());
    Serial.print(' ');
    Serial.print(g_left_wall_sensor);
    Serial.print(' ');
    Serial.print(g_right_wall_sensor);
    Serial.print(' ');
    Serial.print(g_front_wall_sensor);
    Serial.print(' ');
    Serial.print(g_cross_track_error);
    Serial.print(' ');
    Serial.print(g_steering_adjustment);
    Serial.println();
  }
#else
  delay(2);
#endif
}

void report_sensor_track_raw() {
#if DEBUG_LOGGING == 1
  if (millis() >= report_time) {
    report_time += report_interval;
    Serial.print(millis() - start_time);
    Serial.print(' ');
    Serial.print(robot_position());
    Serial.print(' ');
    Serial.print(robot_angle());
    Serial.print(' ');
    Serial.print(g_left_wall_sensor_raw);
    Serial.print(' ');
    Serial.print(g_right_wall_sensor_raw);
    Serial.print(' ');
    Serial.print(g_front_wall_sensor_raw);
    Serial.print(' ');
    Serial.print(g_cross_track_error);
    Serial.print(' ');
    Serial.print(g_steering_adjustment);
    Serial.println();
  }
#else
  delay(2);
#endif
}

void report_front_sensor_track_header() {
#if DEBUG_LOGGING == 1
  Serial.println(F("time pos front_normal front_raw"));
  start_time = millis();
  report_time = start_time;
#endif
}

void report_front_sensor_track() {
#if DEBUG_LOGGING == 1
  if (millis() >= report_time) {
    report_time += report_interval;
    Serial.print(millis() - start_time);
    Serial.print(' ');
    Serial.print(fabsf(robot_position()));
    Serial.print(' ');
    Serial.print(g_front_wall_sensor);
    Serial.print(' ');
    Serial.print(g_front_wall_sensor_raw);
    Serial.println();
  }
#else
  delay(2);
#endif
}

//***************************************************************************//

void report_encoder_header() {
#if DEBUG_LOGGING == 1
  Serial.println(F("left right position angle"));
  start_time = millis();
  report_time = start_time;
#endif
}

void report_encoders() {
#if DEBUG_LOGGING == 1
  if (millis() >= report_time) {
    report_time += report_interval;
    Serial.print(encoder_left_total());
    Serial.print(' ');
    Serial.print(encoder_right_total());
    Serial.print(' ');
    Serial.print(int(robot_position()));
    Serial.print(' ');
    Serial.print(int(robot_angle()));
    Serial.println();
  }
#else
  delay(2);
#endif
}

//***************************************************************************//

void report_pose() {
#if DEBUG_LOGGING == 1
  Serial.print(F("   Angle (deg): "));
  Serial.print(robot_angle());
  Serial.print(F(" Position (mm): "));
  Serial.print(robot_position());
  Serial.println();
  Serial.print(F(" fwd : "));
  Serial.print(forward.position());
  Serial.print(F(" rot : "));
  Serial.print(rotation.position());
  Serial.println();
#else
  delay(2);
#endif
}

//***************************************************************************//

void report_wall_sensors() {
  int left_raw;
  int front_raw;
  int right_raw;
  int left;
  int front;
  int right;
 cli();
    left = g_left_wall_sensor;
    front = g_front_wall_sensor;
    right = g_right_wall_sensor;
    left_raw = g_left_wall_sensor_raw;
    front_raw = g_front_wall_sensor_raw;
    right_raw = g_right_wall_sensor_raw;
  sei();
  Serial.print('\n');
  Serial.print(left);
  Serial.print('(');
  Serial.print(left_raw);
  Serial.print(')');
  Serial.print(' ');
  Serial.print(front);
  Serial.print('(');
  Serial.print(front_raw);
  Serial.print(')');
  Serial.print(' ');
  Serial.print(right);
  Serial.print('(');
  Serial.print(right_raw);
  Serial.print(')');
  Serial.print(' ');
  Serial.print(g_cross_track_error);
  Serial.print(' ');
}

//***************************************************************************//

// simple formatting functions for printing maze costs
void print_hex_2(unsigned char value) {
  if (value < 16) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void print_justified(int value, int width) {
  int v = value;
  int w = width;
  w--;
  if (v < 0) {
    w--;
  }
  while (v /= 10) {
    w--;
  }
  while (w > 0) {
    Serial.write(' ');
    --w;
  }
  Serial.print(value);
}

/***
 * printing functions.
 * Code space can be saved here by not usingserial.print
 */

