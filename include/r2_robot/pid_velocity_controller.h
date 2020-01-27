#include <gazebo_plugins/gazebo_ros_utils.h>

class PIDController {

	double pid_p_;
	double pid_i_;
	double pid_d_;
	double Int_e_;
	double last_e_;

	double max_c_;
	double min_c_;

	double *pos_;

	double PID(double e, double dt) {
		double de = (this->last_e_ - e) / dt;
		this->Int_e_ += e * dt;
		this->last_e_ = e;

		double c = e * this->pid_p_ + this->Int_e_ * this->pid_i_ + de * this->pid_d_;

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

public:
	void init(double *pos, double max_c, double min_c, double kp, double ki, double kd) {
		this->max_c_ = max_c;
		this->min_c_ = min_c;

		this->pid_p_ = kp;
		this->pid_i_ = ki;
		this->pid_d_ = kd;

		this->Int_e_ = 0;
		this->last_e_ = 0;

		this->pos_ = pos;
	}

	void update(gazebo::physics::JointPtr joint, double u, double dt) {
		// double current_pos = this->joint_arm_->Position(0);
		double error = u - *this->pos_;

		double control = this->PID(error, dt);
		joint->SetVelocity(0, control);
		// joint->SetForce(0, control);
	}

	void setK(double kp, double ki, double kd) {
		this->pid_p_ = kp;
		this->pid_i_ = ki;
		this->pid_d_ = kd;
	}
};
