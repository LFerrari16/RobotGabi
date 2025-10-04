/*
 * motor_drive.h
 *
 * Created on: Feb 21, 2024
 * Author: Marto
 * Modified for 2-pin (PWM/DIR) driver.
 */

#ifndef INC_MOTOR_DRIVE_H_
#define INC_MOTOR_DRIVE_H_

// ... (Las macros y defines iniciales no cambian) ...
#define PID_KP	0.5
#define PID_KI	0.01
#define PID_KD	0
#define PID_BIAS	0
#define PID_SAMPLE_TIME_MS	10
#define MAX_CV_VAL		100
#define MIN_PWM_DUTY	4
#define INIT_SPEED		250
#define INIT_ACCEL		800
// Phsyisical params:
#define WHEEL_DIAM_MM	30.5
#define ENCODER_PPR		1080
// Wall Following PID constants (require tuning)
#define WALL_PID_KP 0.05
#define WALL_PID_KI 0
#define WALL_PID_KD 0.002
#define WALL_DISTANCE_SETPOINT_MM 74.0f // Desired distance to a single wall
#define WALL_THRESHOLD_MM       150.0f // Wall detection threshold for side sensors
#define WALL_SENSOR_FILTER_SIZE 5      // Number of samples to average for wall sensors
#define SENSOR_READ_INTERVAL_TICKS 3   // Read sensors every N PID ticks (N * 10ms)
// Directions:
#define LEFT	0
#define RIGHT	1
#define BOTH	2

// New defines for angle PID
#define ANGLE_PID_KP        0.35
#define ANGLE_PID_KI        0
#define ANGLE_PID_KD        0
#define ANGLE_SLOWDOWN_ZONE_DEG 20.0
#define ANGLE_PID_MAX_CV_FAST   70.0
#define ANGLE_PID_MIN_CV_TURN   20.0


#include <math.h>
#include "tim.h"
#include "VL6180X_Array.h"
#include "gpio.h"
#include "pid.h"

struct PidData
{
	double pv_l;
	double sp_l;
	double cv_l;
	double pv_r;
	double sp_r;
	double cv_r;
};
typedef struct PidData PidData_t;

enum StateMachine {I, II, III, IV, V};
typedef enum StateMachine StateMachine_t;

enum MoveState {
	MOTOR_IDLE,
	MOVING_FWD,
	TURNING
};

class MotorDrive
{
	private:
		// PWM timer vars & gpio:
		TIM_HandleTypeDef *pwm_l_tim;
		TIM_HandleTypeDef *pwm_r_tim;
		uint16_t pwm_l_timch;
		uint16_t pwm_r_timch;
		float ccr_scale;
		// CAMBIO: Se reemplazan IN1/IN2 por un solo pin de dirección (DIR)
		GPIO_TypeDef *dir_l_port;
		uint16_t dir_l_pin;
		GPIO_TypeDef *dir_r_port;
		uint16_t dir_r_pin;
		// PID:
		bool pid_stop;
		PID *Pid_l;
		PID *Pid_r;
		PID *Pid_angle;
		double pid_l_sp;
		double pid_l_cv;
		double pid_l_pv;
		double final_pos_l;
		double pid_r_sp;
		double pid_r_cv;
		double pid_r_pv;
		double final_pos_r;
		float current_speed; // Current speed for ramp generator (mm/s or deg/s)
		double angle_pv, angle_sp, angle_cv;
		int speed;
		int accel;
		double bias;
		float pos_inc_l;
		float pos_inc_r;
		float mm_per_pulse;
		bool pid_flag=false;
		// Wall Following PID:
		VL6180X_Array* p_tof_sensors;
		PID            *Pid_wall;
		double         wall_pid_pv;
		double         wall_pid_cv;
		double         wall_pid_sp;
		MoveState move_state;
		float          wall_distance_setpoint;
		bool           wall_following_enabled;
		uint16_t       wall_sensor_left_hist[WALL_SENSOR_FILTER_SIZE];
		uint16_t       wall_sensor_right_hist[WALL_SENSOR_FILTER_SIZE];
		uint8_t        wall_sensor_hist_idx;
		float          last_known_right_dist_mm;
		float          last_known_left_dist_mm;
		// Internal methods:
		void _dir_fwd_l();
		void _dir_rev_l();
		void _dir_fwd_r();
		void _dir_rev_r();
		void set_pwm_duty(double,double);

	public:
		// CAMBIO: Constructor modificado para pines DIR
		MotorDrive(TIM_HandleTypeDef* pwm_l, uint16_t pwm_ch_l, GPIO_TypeDef* dir_l_port, uint16_t dir_l_pin,
				   TIM_HandleTypeDef* pwm_r, uint16_t pwm_ch_r, GPIO_TypeDef* dir_r_port, uint16_t dir_r_pin,
				   int speed, int accel, VL6180X_Array* sensors);

		void move_forward(int,uint32_t);
		void turn(int angle_deg, uint32_t timeout_ms);
		void stop_coast();
		void stop_brake();
		void set_wall_following(bool enable);
		void set_sp(int,int);
		void set_speed(int);
		void set_accel(int);
		void reset_pid(int);
		void set_kp(double);
		void set_ki(double);
		void set_kd(double);
		void set_wall_kp(double);
		void set_wall_ki(double);
		void set_wall_kd(double);
		void set_angle_kp(double);
		void set_angle_ki(double);
		void set_angle_kd(double);
		void set_bias(double);
		void enc_l_inc();
		void enc_l_dec();
		void enc_r_inc();
		void enc_r_dec();
		void update_pid();
		PidData_t get_pid_data();
};

#endif /* INC_MOTOR_DRIVE_H_ */