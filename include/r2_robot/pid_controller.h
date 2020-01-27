class PIDController {

	float pid_p_;
	float pid_i_;
	float pid_d_;
	float Int_e_;
	float last_e_;

	float max_c_;
	float min_c_;

	float *pos_;

	float PID(float e, float dt);

public:
	PIDController(float *pos, float max_c, float min_c, float kp, float ki, float kd);
	float update(float u, float dt);
	void setK(float kp, float ki, float kd);
};
