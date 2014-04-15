#include "CheezyDrive.h"
#include <math.h>
CheezyDrive::CheezyDrive(){
  old_wheel_ = 0.0;
  out_r = 0.0;
  quickStopAccumulator_ = 0.0;
  out_l = 0.0;
}
CheezyDrive::~CheezyDrive(){

}
void CheezyDrive::updateTank(float r_, float l_, bool quickTurn, bool highGear){
  float throttle = ( r_ + l_ ) / 2.0;
  float ang = (r_ - l_) / 2.0;
  update(throttle, ang, quickTurn, highGear);
}
void CheezyDrive::update(float throttle, float wheel, bool quickTurn, bool highGear){
  bool isQuickTurn = quickTurn;
  bool isHighGear = highGear;

  double wheelNonLinearity;

  double neg_inertia = wheel - old_wheel_;
  old_wheel_ = wheel;

  double M_PI = 3.141592;

  if (isHighGear) {
    wheelNonLinearity = 0.95;
    // Apply a sin function that's scaled to make it feel better.
    wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
    wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
  } else {
    wheelNonLinearity = 0.9;
    // Apply a sin function that's scaled to make it feel better.
    wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
    wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
    wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
  }

  double left_pwm, right_pwm, overPower;
  float sensitivity = 1.55;

  float angular_power;
  float linear_power;

  // Negative inertia!
  static double neg_inertia_accumulator = 0.0;
  //printf("High gear is %d\n", isHighGear);
  double neg_inertia_scalar;
  if (isHighGear) {
    neg_inertia_scalar = 8.0;
    sensitivity = 1.2;
  } else {
    if (wheel * neg_inertia > 0) {
      neg_inertia_scalar = 2.5;
    } else {
      if (fabs(wheel) > 0.65) {
        neg_inertia_scalar = 5.0;
      } else {
        neg_inertia_scalar = 3.0;
      }
    }
    sensitivity =  1.1;

    if (fabs(throttle) > 0.1) {
      sensitivity = 1 - (1 - sensitivity) / fabs(throttle);
    }
  }
  double neg_inertia_power = neg_inertia * neg_inertia_scalar;
  neg_inertia_accumulator += neg_inertia_power;

  wheel = wheel + neg_inertia_accumulator;
  if(neg_inertia_accumulator > 1)
    neg_inertia_accumulator -= 1;
  else if (neg_inertia_accumulator < -1)
    neg_inertia_accumulator += 1;
  else
    neg_inertia_accumulator = 0;

  linear_power = throttle;

  // Quickturn!
  if (isQuickTurn) {
    if (fabs(linear_power) < 0.2) {
      double alpha =  0.1;
      wheel = (fabs(wheel) > 1.0)? wheel / fabs(wheel) : wheel;
    }
    overPower = 1.0;
    if (isHighGear) {
      sensitivity = 1.0;
    } else {
      sensitivity = 1.0;
    }
    angular_power = wheel;
  } else {
    overPower = 0.0;
    angular_power = fabs(throttle) * wheel * sensitivity;
  }

  right_pwm = left_pwm = linear_power;
  left_pwm += angular_power;
  right_pwm -= angular_power;

  if (left_pwm > 1.0) {
    right_pwm -= overPower * (left_pwm - 1.0);
    left_pwm = 1.0;
  } else if (right_pwm > 1.0) {
    left_pwm -= overPower * (right_pwm - 1.0);
    right_pwm = 1.0;
  } else if (left_pwm < -1.0) {
    right_pwm += overPower * (-1.0 - left_pwm);
    left_pwm = -1.0;
  } else if (right_pwm < -1.0) {
    left_pwm += overPower * (-1.0 - right_pwm);
    right_pwm = -1.0;
  }

  //printf("left pwm: %f right pwm: %f\n", left_pwm, right_pwm);
  out_r = right_pwm; 
  out_l = -left_pwm;
}
float CheezyDrive::get_r(){
  return out_r; 
}
float CheezyDrive::get_l(){
  return out_l;
}
