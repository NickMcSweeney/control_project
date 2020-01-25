#include "r2_robot/pid_controller.h"

PIDController::PIDController(float *pos, float max_c, float min_c, float kp, float ki, float kd) {
  this->max_c_ = max_c;
  this->min_c_ = min_c;

  this->pid_p_ = kp;
  this->pid_i_ = ki;
  this->pid_d_ = kd;

  this->Int_e_ = 0;
  this->last_e_ = 0;

  this->pos_ = pos;
}

float PIDController::PID(float e, float dt) {
  float de = (this->last_e_ - e) / dt;
  this->Int_e_ += e * dt;
  this->last_e_ = e;

  float c = e * this->pid_p_ + this->Int_e_ * this->pid_i_ + de * this->pid_d_;

  // Clamp c values (velocity)
  if (c > this->max_c_)
    return this->max_c_;
  else if (c < this->min_c_ && c > -this->min_c_)
    return 0;
  else if (c < -this->max_c_)
    return -this->max_c_;
  else
    return c;
}

void PIDController::update(gazebo::physics::JointPtr joint, float u, float dt) {
  // float current_pos = this->joint_arm_->Position(0);
  float error = u - *this->pos_;

  float control = this->PID(error, dt);
  joint->SetVelocity(0, control);
  // this->joint_arm_->SetForce(0, control);
}

void PIDController::setK(float kp, float ki, float kd) {
  this->pid_p_ = kp;
  this->pid_i_ = ki;
  this->pid_d_ = kd;
}
