/*
 * File: motors.cpp
 * Project: mazerunner
 * File Created: Monday, 29th March 2021 11:05:58 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Monday, 5th April 2021 3:01:38 pm
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

#include "motors.h"

#include "encoders.h"
#include"config.h"
#include "profile.h"
#include "sensors.h"
#include <Arduino.h>
gpio_config_t io_conf;
// these are maintained only for logging
float g_left_motor_volts;
float g_right_motor_volts;

static bool s_controllers_output_enabled;
static float s_old_fwd_error;
static float s_old_rot_error;
static float s_fwd_error;
static float s_rot_error;
Profile forward;
Profile rotation;

void enable_motor_controllers() {
  s_controllers_output_enabled = true;
}

void disable_motor_controllers() {
  s_controllers_output_enabled = false;
}

void reset_motor_controllers() {
  s_fwd_error = 0;
  s_rot_error = 0;
  s_old_fwd_error = 0;
  s_old_rot_error = 0;
}

void setup_motors() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pin_bit_mask = (1 << MOTOR_LEFT_DIR) | (1 << MOTOR_RIGHT_DIR); 
  gpio_config(&io_conf);
  set_motor_pwm_frequency(mor_freq);
  stop_motors();
}

float position_controller() {
  s_fwd_error += forward.increment() - robot_fwd_increment();
  float diff = s_fwd_error - s_old_fwd_error;
  s_old_fwd_error = s_fwd_error;
  float output =FWD_KP * s_fwd_error +FWD_KD * diff;
  return output;
}

float angle_controller(float steering_adjustment) {
  s_rot_error += rotation.increment() - robot_rot_increment();
  if (g_steering_enabled) {
    s_rot_error += steering_adjustment;
  }
  float diff = s_rot_error - s_old_rot_error;
  s_old_rot_error = s_rot_error;
  float output = ROT_KP * s_rot_error + ROT_KD * diff;
  return output;
}
void update_motor_controllers(float steering_adjustment) {
  float pos_output = position_controller();
  float rot_output = angle_controller(steering_adjustment);
  float left_output = 0;
  float right_output = 0;
  left_output += pos_output;
  right_output += pos_output;
  left_output -= rot_output;
  right_output += rot_output;
  float v_fwd = forward.speed();
  float v_rot = rotation.speed();
  float v_left = v_fwd - (PI / 180.0) * MOUSE_RADIUS * v_rot;
  float v_right = v_fwd + (PI / 180.0) * MOUSE_RADIUS * v_rot;
  left_output += SPEED_FF * v_left;
  right_output += SPEED_FF * v_right;
  left_output=constrain(left_output,0,255);
  right_output=constrain(right_output,0,255);
  if (s_controllers_output_enabled) {
    set_left_motor_pwm(left_output);
    set_right_motor_pwm(right_output);
  }
}
/**
 * Direct register access could be used here for enhanced performance
 */
void set_left_motor_pwm(int pwm) {
  pwm =constrain(pwm, 0, 255);
  if (pwm == 0) {
     GPIO.out_w1tc = (1 << MOTOR_LEFT_DIR) ;
    ledcWrite(left_Channel, 0);
  } else {
    GPIO.out_w1ts = (1 << MOTOR_LEFT_DIR) ;
    ledcWrite(left_Channel, pwm);
  }
}

void set_right_motor_pwm(int pwm) {
  pwm =constrain(pwm, 0, 255);
  if (pwm == 0) {
     GPIO.out_w1tc = (1 << MOTOR_RIGHT_DIR);
    ledcWrite(right_Channel, 0);
  } else {
     GPIO.out_w1ts = (1 << MOTOR_RIGHT_DIR);
    ledcWrite(right_Channel, pwm);
  }
}



void set_motor_pwm_frequency(int frequency) {
  ledcSetup(left_Channel, frequency, resolution);
  ledcSetup(right_Channel, frequency, resolution);
  ledcAttachPin(MOTOR_LEFT_PWM, left_Channel);
  ledcAttachPin(MOTOR_RIGHT_PWM, right_Channel);
}

void stop_motors() {
  set_left_motor_pwm(0);
  set_right_motor_pwm(0);
}

/****************************************************************************/
