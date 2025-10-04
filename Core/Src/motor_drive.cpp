/*
 * motor_drive.cpp
 *
 *  Created on: Feb 21, 2024
 *      Author: Marto
 */

#include "motor_drive.h"
#include <math.h>

/*	Function: MotorDrive (constructor)
	--------------------
	Setup private vars, including timers and gpio handles. Based on TB6612FNG IC motor driver

	pwm_l: timer handle used to create PWM signal (Left motor)
	pwm_ch_l: PWM timer channel number (Left motor)
	in1_l_port_p: output port handle for IN1 signal (Left motor)
	in1_l_pin_p: output pin number for IN1 signal (Left motor)
	in2_l_port_p: Same for IN2 signal
	in2_l_pin_p: Same for IN2 signal
	speed_p: robot speed, in mm/sec
 */
MotorDrive::MotorDrive(
		TIM_HandleTypeDef* pwm_l,
		uint16_t pwm_ch_l,
		GPIO_TypeDef* in1_l_port_p,
		uint16_t in1_l_pin_p,
		GPIO_TypeDef* in2_l_port_p,
		uint16_t in2_l_pin_p,
		TIM_HandleTypeDef* pwm_r,
		uint16_t pwm_ch_r,
		GPIO_TypeDef* in1_r_port_p,
		uint16_t in1_r_pin_p,
		GPIO_TypeDef* in2_r_port_p,
		uint16_t in2_r_pin_p,
		L3GD20* gyro_p,
		int speed_p,
		int accel_p,
		VL53L0X_array* sensors_p
		)
{
	// PWM:
	pwm_l_tim = pwm_l;
	pwm_l_timch = pwm_ch_l;
	pwm_r_tim = pwm_r;
	pwm_r_timch = pwm_ch_r;
	ccr_scale = (pwm_l_tim->Init.Period+1) /100;
	// Motor driver:
	in1_l_port = in1_l_port_p;
	in1_l_pin = in1_l_pin_p;
	in2_l_port = in2_l_port_p;
	in2_l_pin = in2_l_pin_p;
	in1_r_port = in1_r_port_p;
	in1_r_pin = in1_r_pin_p;
	in2_r_port = in2_r_port_p;
	in2_r_pin = in2_r_pin_p;
	gyro = gyro_p;
	// PID:
	Pid_l = new PID(&pid_l_pv, &pid_l_cv, &pid_l_sp, PID_KP, PID_KI, PID_KD, P_ON_E, DIRECT);
	Pid_l->SetSampleTime(PID_SAMPLE_TIME_MS);
	Pid_l->SetMode(AUTOMATIC);
	Pid_l->SetOutputLimits(-MAX_CV_VAL, MAX_CV_VAL);
	Pid_r = new PID(&pid_r_pv, &pid_r_cv, &pid_r_sp, PID_KP, PID_KI, PID_KD, P_ON_E, DIRECT);
	Pid_r->SetSampleTime(PID_SAMPLE_TIME_MS);
	Pid_r->SetMode(AUTOMATIC);
	Pid_r->SetOutputLimits(-MAX_CV_VAL, MAX_CV_VAL);
	reset_pid(BOTH);	
	Pid_angle = new PID(&angle_pv, &angle_cv, &angle_sp, ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD, P_ON_E, DIRECT);
	Pid_angle->SetSampleTime(PID_SAMPLE_TIME_MS);
	Pid_angle->SetMode(MANUAL);
	Pid_angle->SetOutputLimits(-ANGLE_PID_MAX_CV_FAST, ANGLE_PID_MAX_CV_FAST);
	// Wall Following PID setup
	p_tof_sensors = sensors_p;
	wall_following_enabled = false;
	Pid_wall = new PID(&wall_pid_pv, &wall_pid_cv, &wall_pid_sp, WALL_PID_KP, WALL_PID_KI, WALL_PID_KD, P_ON_E, REVERSE);
	Pid_wall->SetSampleTime(PID_SAMPLE_TIME_MS);
	Pid_wall->SetOutputLimits(-50, 50); // Limit the correction PWM value
	Pid_wall->SetMode(AUTOMATIC);
	// Initialize wall following state
	wall_distance_setpoint = WALL_DISTANCE_SETPOINT_MM; // Default value
	last_known_left_dist_mm = WALL_DISTANCE_SETPOINT_MM;
	last_known_right_dist_mm = WALL_DISTANCE_SETPOINT_MM;
	// Initialize wall sensor history
	wall_sensor_hist_idx = 0;
	for (int i = 0; i < WALL_SENSOR_FILTER_SIZE; ++i) {
		wall_sensor_left_hist[i] = 0;
		wall_sensor_right_hist[i] = 0;
	}

	move_state = MOTOR_IDLE;

	mm_per_pulse = WHEEL_DIAM_MM*3.1416/ENCODER_PPR;
	set_speed(speed_p);
	set_accel(accel_p);
	bias = PID_BIAS;

	stop_coast();	// Start with motor stopped
	HAL_TIM_PWM_Start(pwm_l_tim, pwm_l_timch); // Start PWM channels
	HAL_TIM_PWM_Start(pwm_r_tim, pwm_r_timch);
}

/*
	Function: move_forward
	--------------------
	Sets pwm and output pins to move the robot forward, and starts the PWM timer

	pwm_duty: duty cycle, in %
 */
void MotorDrive::move_forward(int sp, uint32_t timeout_ms)
{
	StateMachine_t state = I;
	bool move_finished;
	uint32_t start_t, last_ramp_update_time;
	int pid_tick_counter = 0;

//	_dir_fwd_l();
//	_dir_fwd_r();
//	set_pwm_duty(25,50);
//	HAL_Delay(2000);
//	HAL_GPIO_WritePin(in1_l_port, in1_l_pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(in2_l_port, in2_l_pin, GPIO_PIN_RESET);
//	HAL_Delay(2000);
//	HAL_GPIO_WritePin(in1_r_port, in1_r_pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(in2_r_port, in2_r_pin, GPIO_PIN_RESET);
//	_dir_rev_l();
//	_dir_rev_r();
//	HAL_Delay(2000);
//	HAL_GPIO_WritePin(in1_l_port, in1_l_pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(in2_l_port, in2_l_pin, GPIO_PIN_RESET);
//	return;

	// Setup ramp generator:
	current_speed = 0;
	pos_inc_l = 0;
	pos_inc_r = 0;

	// Setup PID:
	reset_pid(BOTH);
	Pid_l->SetMode(AUTOMATIC);
	Pid_r->SetMode(AUTOMATIC);

	move_state = MOVING_FWD;
	set_sp(sp, sp);
	move_finished = false;
	start_t = HAL_GetTick();
	last_ramp_update_time = start_t;
	pid_stop = false;	// Enable PID update interrupt
	while(move_finished == false)
	{
		// Check timeout:
		if(timeout_ms>0 and (HAL_GetTick()-start_t)>timeout_ms)
			break;

		// Sensor reading is timed by the PID flag to ensure a predictable rate
		// and to avoid blocking the main thread excessively, which causes encoder pulse loss.
		if (pid_flag && wall_following_enabled)
		{
			pid_tick_counter++;
			if (pid_tick_counter >= SENSOR_READ_INTERVAL_TICKS) {
				pid_tick_counter = 0;
				p_tof_sensors->ping_single(0); // Right sensor
				p_tof_sensors->ping_single(3); // Left sensor
			}
		}

		// Executes once every PID interrupt:
		if(pid_flag)
		{
			pid_flag = false;
			uint32_t now = HAL_GetTick();
			float dt_sec = (now - last_ramp_update_time) / 1000.0f;
			last_ramp_update_time = now;
			switch(state)
			{
				// Speed up ramp:
				case I:
					current_speed += accel * dt_sec;
					if(current_speed >= speed)
					{
						current_speed = speed;
						state = II;
					}
					// Calcs using "left" vars, then copyes to the "right", since forward
					// ramp is the same for both motors.
					pos_inc_l = current_speed * dt_sec;
					pos_inc_r = pos_inc_l;
					pid_l_sp += pos_inc_l;
					pid_r_sp = pid_l_sp;
					break;
				// Constant speed:
				case II:
					pos_inc_l = current_speed * dt_sec;
					pos_inc_r = pos_inc_l;
					pid_l_sp += pos_inc_l;
					if(pid_l_sp >= final_pos_l)
					{
						pid_l_sp = final_pos_l;
						state = III;
					}
					pid_r_sp = pid_l_sp;
					break;
				// Wait for PV to reach final value at const. speed:
				case III:
					// Stop ramp generation, just wait for the robot to reach the final setpoint
					pos_inc_l = 0;
					pos_inc_r = 0;
					if(pid_l_pv >= final_pos_l)
						move_finished = true;
					break;
			}
		}
	}
	//HAL_Delay(3000);
	stop_coast();
}

void MotorDrive::turn(int angle_deg, uint32_t timeout_ms)
{
	StateMachine_t state = I;
	bool turn_finished = false;

	// Set state
	uint32_t start_t, last_ramp_update_time;
	float angle_inc;
	float final_angle = angle_deg;
	float decel_angle_start;

	// Setup ramp generator for angular speed.
	// Using linear speed and accel members as deg/s and deg/s^2.
	current_speed = 0.0f;
	// Reset angle measurement and PID
	angle_pv = 0;
	angle_sp = 0; // Setpoint starts at 0 and will be ramped
	angle_cv = 0;
	Pid_angle->Initialize();
	Pid_angle->SetMode(AUTOMATIC);
	// Set output limits to max for the whole turn, PID will handle the output
	Pid_angle->SetOutputLimits(-ANGLE_PID_MAX_CV_FAST, ANGLE_PID_MAX_CV_FAST);

	// Stop distance PID controllers
	Pid_l->SetMode(MANUAL);
	Pid_r->SetMode(MANUAL);

	// Set state
	move_state = TURNING;
	
	start_t = HAL_GetTick();
	last_ramp_update_time = start_t;
	pid_stop = false; // Enable PID updates

	while(!turn_finished)
	{
		// Check timeout
		if(timeout_ms > 0 && (HAL_GetTick() - start_t) > timeout_ms)
		{
			break;
		}

		// pid_flag is set by the timer interrupt that calls update_pid
		if(pid_flag)
		{
			pid_flag = false;
			uint32_t now = HAL_GetTick();
			float dt_sec = (now - last_ramp_update_time) / 1000.0f;
			last_ramp_update_time = now;
			// Check if turn is complete. Stop when SP is reached, ignore overshoot.
			switch(state)
			{
				case I: // Acceleration phase
				{
					current_speed += accel * dt_sec;
					if (current_speed >= speed)
					{
						current_speed = speed;
						state = II; // Move to constant speed
					}

					// Calculate the angle needed to decelerate from current speed
					float decel_dist = (current_speed * current_speed) / (2.0f * accel);
					if (final_angle > 0) {
						decel_angle_start = final_angle - decel_dist;
						if (angle_sp >= decel_angle_start) { state = III; } // Check if we need to start decelerating
					} else { // Negative angle
						decel_angle_start = final_angle + decel_dist;
						if (angle_sp <= decel_angle_start) { state = III; } // Check if we need to start decelerating
					}

					angle_inc = current_speed * dt_sec;
					angle_sp += (final_angle > 0) ? angle_inc : -angle_inc;
					break;
				}

				case II: // Constant speed phase
				{
					float decel_dist = (current_speed  * current_speed ) / (2.0f * accel);
					if (final_angle > 0) {
						decel_angle_start = final_angle - decel_dist;
						if (angle_sp >= decel_angle_start) { state = III; }
					} else {
						decel_angle_start = final_angle + decel_dist;
						if (angle_sp <= decel_angle_start) { state = III; }
					}

					angle_inc = current_speed  * dt_sec;
					angle_sp += (final_angle > 0) ? angle_inc : -angle_inc;
					break;
				}

				case III: // Deceleration phase
				{
					current_speed  -= accel * dt_sec;
					if (current_speed  < 5.0f) // Use a minimum speed to avoid stalling (e.g. 5 deg/s)
					{
						current_speed  = 5.0f;
					}

					angle_inc = current_speed  * dt_sec;
					angle_sp += (final_angle > 0) ? angle_inc : -angle_inc;

					if ((final_angle > 0 && angle_sp >= final_angle) || (final_angle < 0 && angle_sp <= final_angle))
					{
						angle_sp = final_angle;
						state = IV; // Wait for completion
					}
					break;
				}

				case IV: // Wait for completion
				{
					if (fabs(final_angle - angle_pv) < 1.5) // 1.5 degree tolerance
					{
						turn_finished = true;
					}
					break;
				}
			}

			// Ensure setpoint doesn't overshoot the final angle
			if ((final_angle > 0 && angle_sp > final_angle) || (final_angle < 0 && angle_sp < final_angle))
			{
				angle_sp = final_angle;
			}
		}
	}

	stop_coast(); // This will set move_state to MOTOR_IDLE
	Pid_angle->SetMode(MANUAL); // Disable angle PID
}

/*
	Function: stop_coast
	--------------------
	Sets output pins to stop the robot (coast), and stops the PWM timer
 */
void MotorDrive::stop_coast()
{
	//HAL_TIM_PWM_Stop(pwm_l_tim, pwm_l_timch);
	pid_stop = true;
	wall_following_enabled = false;
	move_state = MOTOR_IDLE;
	HAL_GPIO_WritePin(in1_l_port, in1_l_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(in2_l_port, in2_l_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(in1_r_port, in1_r_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(in2_r_port, in2_r_pin, GPIO_PIN_RESET);
}

/*
	Function: stop_brake
	--------------------
	Sets output pins to stop the robot (brake), and stops the PWM timer
 */
void MotorDrive::stop_brake()
{
	//HAL_TIM_PWM_Stop(pwm_l_tim, pwm_l_timch);
	pid_stop = true;
	wall_following_enabled = false;
	move_state = MOTOR_IDLE;
	HAL_GPIO_WritePin(in1_l_port, in1_l_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(in2_l_port, in2_l_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(in1_r_port, in1_r_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(in2_r_port, in2_r_pin, GPIO_PIN_SET);
}

/*
	Function: set_pwm_duty
	--------------------
	Sets individual duty value in percentage (0-100) for both pwm outputs

	duty_r: duty cycle for left motor, in %
	duty_l: duty cycle for right motor, in %
 */
void MotorDrive::set_pwm_duty(double duty_r, double duty_l)
{
	uint16_t final_duty_l, final_duty_r;

	// Left Motor
	duty_l = fabs(duty_l); // Get absolute value
	if (duty_l > 100.0) duty_l = 100.0; // Cap at 100%

	if (duty_l > 0.0) { // Only apply dead-zone compensation if motor should be moving
		duty_l += MIN_PWM_DUTY;
		if (duty_l > 100.0) duty_l = 100.0; // Recalculate cap
	}
	final_duty_l = (uint16_t)(duty_l * ccr_scale);
	__HAL_TIM_SET_COMPARE(pwm_l_tim, pwm_l_timch, final_duty_l);

	// Right Motor
	duty_r = fabs(duty_r); // Get absolute value
	if (duty_r > 100.0) duty_r = 100.0; // Cap at 100%

	if (duty_r > 0.0) { // Only apply dead-zone compensation if motor should be moving
		duty_r += MIN_PWM_DUTY;
		if (duty_r > 100.0) duty_r = 100.0; // Recalculate cap
	}
	final_duty_r = (uint16_t)(duty_r * ccr_scale);
	__HAL_TIM_SET_COMPARE(pwm_r_tim, pwm_r_timch, final_duty_r);
}

/*
	Function: set_sp
	--------------------
	Sets final SP value for selected PID

	dir: select which PID controller (LEFT or RIGTH)
	value: SP position value, in mm
 */
void MotorDrive::set_sp(int right, int left)
{
	final_pos_l = left;
	final_pos_r = right;
}

/*
	Function: set_speed
	--------------------
	Sets robot speed

	value: speed value, in mm/s
 */
void MotorDrive::set_speed(int value)
{
	if(value>0)
		speed = value;
}

/*
	Function: set_accel
	--------------------
	Sets robot acceleration

	value: accel. value, in mm/s^2
 */
void MotorDrive::set_accel(int value)
{
	if(value>0)
		accel = value;
}


/*
	Function: set_kp
	--------------------
	Sets PID kp value

	value: kp value
 */
void MotorDrive::set_kp(double value)
{
	if(value>=0)
	{
		double ki = Pid_l->GetKi();
		double kd = Pid_l->GetKd();
		Pid_l->SetTunings(value, ki, kd);
		Pid_r->SetTunings(value, ki, kd);
	}
}

/*
	Function: set_ki
	--------------------
	Sets PID ki value

	value: ki value
 */
void MotorDrive::set_ki(double value)
{
	if(value>=0)
	{
		double kp = Pid_l->GetKp();
		double kd = Pid_l->GetKd();
		Pid_l->SetTunings(kp, value, kd);
		Pid_r->SetTunings(kp, value, kd);
	}
}

/*
	Function: set_kd
	--------------------
	Sets PID kd value

	value: kd value
 */
void MotorDrive::set_kd(double value)
{
	if(value>=0)
	{
		double ki = Pid_l->GetKi();
		double kp = Pid_l->GetKp();
		Pid_l->SetTunings(kp, ki, value);
		Pid_r->SetTunings(kp, ki, value);
	}
}

/*
	Function: set_bias
	--------------------
	Sets PID bias value, used for feedforward control

	value: bias value
 */
void MotorDrive::set_bias(double value)
{
	if(value>=0)
		bias = value;
}

/*
	Function: set_wall_kp
	--------------------
	Sets Wall Following PID kp value

	value: kp value
 */
void MotorDrive::set_wall_kp(double value)
{
	if(value>=0)
	{
		double ki = Pid_wall->GetKi();
		double kd = Pid_wall->GetKd();
		Pid_wall->SetTunings(value, ki, kd);
	}
}

/*
	Function: set_wall_ki
	--------------------
	Sets Wall Following PID ki value

	value: ki value
 */
void MotorDrive::set_wall_ki(double value)
{
	if(value>=0)
	{
		double kp = Pid_wall->GetKp();
		double kd = Pid_wall->GetKd();
		Pid_wall->SetTunings(kp, value, kd);
	}
}

/*
	Function: set_wall_kd
	--------------------
	Sets Wall Following PID kd value

	value: kd value
 */
void MotorDrive::set_wall_kd(double value)
{
	if(value>=0)
	{
		double kp = Pid_wall->GetKp();
		double ki = Pid_wall->GetKi();
		Pid_wall->SetTunings(kp, ki, value);
	}
}


/*
	Function: set_angle_kp
	--------------------
	Sets Angle PID kp value

	value: kp value
 */
void MotorDrive::set_angle_kp(double value)
{
	if(value>=0)
	{
		double ki = Pid_angle->GetKi();
		double kd = Pid_angle->GetKd();
		Pid_angle->SetTunings(value, ki, kd);
	}
}

/*
	Function: set_angle_ki
	--------------------
	Sets Angle PID ki value

	value: ki value
 */
void MotorDrive::set_angle_ki(double value)
{
	if(value>=0)
	{
		double kp = Pid_angle->GetKp();
		double kd = Pid_angle->GetKd();
		Pid_angle->SetTunings(kp, value, kd);
	}
}

void MotorDrive::set_angle_kd(double value)
{
	if(value>=0)
	{
		double kp = Pid_angle->GetKp();
		double ki = Pid_angle->GetKi();
		Pid_angle->SetTunings(kp, ki, value);
	}
}

/*
	Function: set_wall_following
	--------------------
	Enables or disables the wall following logic inside the PID update.
 */
void MotorDrive::set_wall_following(bool enable)
{
	wall_following_enabled = enable;
	if (enable) {
		// When enabling wall following, reset the PID and determine the initial state and setpoint.
		Pid_wall->Initialize();

		// --- Dynamic Setpoint Calculation ---
		// Per user request, we assume the robot starts centered between two walls.
		// We will calculate the ideal center distance by averaging the left and right
		// sensor readings. This value will be the setpoint for single-wall following.
		uint32_t sum_right = 0;
		uint32_t sum_left = 0;
		int valid_readings_right = 0;
		int valid_readings_left = 0;

		for (int i = 0; i < 5; ++i) {
			p_tof_sensors->ping_single(0); // Ping right sensor (index 0)
			p_tof_sensors->ping_single(3); // Ping left sensor (index 3)

			uint16_t reading_right = p_tof_sensors->get_distance(0);
			if (reading_right < WALL_THRESHOLD_MM) {
				sum_right += reading_right;
				valid_readings_right++;
			}
			uint16_t reading_left = p_tof_sensors->get_distance(3);
			if (reading_left < WALL_THRESHOLD_MM) {
				sum_left += reading_left;
				valid_readings_left++;
			}
			HAL_Delay(5); // Small delay between readings
		}

		if (valid_readings_right > 0 && valid_readings_left > 0) {
			// Both walls detected, calculate the center setpoint
			float avg_right = (float)sum_right / valid_readings_right;
			float avg_left = (float)sum_left / valid_readings_left;
			wall_distance_setpoint = (avg_right + avg_left) / 2.0f;
		} else if (valid_readings_right > 0) {
			// Only right wall detected, use it as setpoint (fallback)
			wall_distance_setpoint = (float)sum_right / valid_readings_right;
		} else if (valid_readings_left > 0) {
			// Only left wall detected, use it as setpoint (fallback)
			wall_distance_setpoint = (float)sum_left / valid_readings_left;
		} else {
			// No walls detected, use default
			wall_distance_setpoint = WALL_DISTANCE_SETPOINT_MM;
		}

		// Initialize last known distances. If a wall is present, use its current value.
		// If not, use the calculated or default setpoint. This provides a stable start.
		last_known_right_dist_mm = (valid_readings_right > 0) ? ((float)sum_right / valid_readings_right) : wall_distance_setpoint;
		last_known_left_dist_mm = (valid_readings_left > 0) ? ((float)sum_left / valid_readings_left) : wall_distance_setpoint;

		// Populate the sensor history buffer with initial readings to avoid starting with zeros.
		// This is important for the moving average filter to work correctly from the start.
		// This is a blocking call, so it's done only once on setup.
		// We use the averaged values we just calculated to provide a stable starting point for the filter.
		uint16_t initial_right_avg = (valid_readings_right > 0) ? (uint16_t)((float)sum_right / valid_readings_right) : (uint16_t)last_known_right_dist_mm;
		uint16_t initial_left_avg = (valid_readings_left > 0) ? (uint16_t)((float)sum_left / valid_readings_left) : (uint16_t)last_known_left_dist_mm;


		for (int i = 0; i < WALL_SENSOR_FILTER_SIZE; ++i) {
			wall_sensor_right_hist[i] = initial_right_avg;
			wall_sensor_left_hist[i] = initial_left_avg;
		}
		wall_sensor_hist_idx = 0;
	}
}

/*
	Function: update_pid
	--------------------
	Computes both PID controllers, and updates the corresponding PWM outputs
 */
void MotorDrive::update_pid()
{
	if(pid_stop)
		return;

	pid_flag = true;

	if (move_state == TURNING)
	{
		// Update angle from gyro
		l3gd20Data gyro_data = gyro->read();
		// Integrate angular velocity to get angle. Sample time is PID_SAMPLE_TIME_MS.
		angle_pv += (gyro_data.z * (PID_SAMPLE_TIME_MS / 1000.0));

		// Compute angle PID. The setpoint (angle_sp) is now updated by the turn() function's state machine.
		Pid_angle->Compute();

		// angle_cv will be positive for right turn, negative for left turn.
		// It represents the turning "effort".
		// We can apply this to the wheels differentially.
		if (angle_cv > 0) { // Turn right
			_dir_fwd_r();
			_dir_rev_l();
		} else { // Turn left
			_dir_rev_r();
			_dir_fwd_l();
		}
		set_pwm_duty(angle_cv, angle_cv); // Same duty, opposite directions
	}
	else if (move_state == MOVING_FWD)
	{
		// To decouple distance and steering control, we use a single PID
		// for the robot's average forward distance, and the wall-following PID for steering.

		// 1. Calculate the average distance from both encoders. This represents the
		//    forward travel of the robot's center.
		double average_pv = (pid_l_pv + pid_r_pv) / 2.0;

		// 2. Temporarily use this average as the input for our master distance PID (Pid_l).
		//    We save the original left PV and restore it later to not mess up other logic (like telemetry).
		double original_pid_l_pv = pid_l_pv;
		pid_l_pv = average_pv;

		// 3. Compute the master distance PID. Its setpoint (pid_l_sp) is ramped in move_forward().
		Pid_l->Compute();

		// 4. Restore the original left PV.
		pid_l_pv = original_pid_l_pv;

		// Pid_r is not used for forward movement control to avoid conflicts.

		double wall_correction = 0.0;
		if (wall_following_enabled)
		{
			// --- Sensor reading and filtering ---
			// The sensor pings are now done in the main loop (inside move_forward).
			// Here, we just get the latest values and update the filter.
			// Note: get_distance() returns the last measured value, it does not trigger a new measurement.
			uint16_t raw_right_dist = p_tof_sensors->get_distance(0);
			uint16_t raw_left_dist = p_tof_sensors->get_distance(3);

			// --- Intersection-proof Filtering ---
			// Check for wall presence based on raw sensor data.
			bool right_wall_present = raw_right_dist < WALL_THRESHOLD_MM;
			bool left_wall_present  = raw_left_dist  < WALL_THRESHOLD_MM;

			// If a wall is present, use its value for the filter and update the last known good distance.
			// If a wall is NOT present (intersection), feed the filter with the last known good distance.
			// This prevents large out-of-range values (8191) from polluting the moving average.
			if (right_wall_present) {
				wall_sensor_right_hist[wall_sensor_hist_idx] = raw_right_dist;
				last_known_right_dist_mm = (float)raw_right_dist;
			} else {
				wall_sensor_right_hist[wall_sensor_hist_idx] = (uint16_t)last_known_right_dist_mm;
			}

			if (left_wall_present) {
				wall_sensor_left_hist[wall_sensor_hist_idx] = raw_left_dist;
				last_known_left_dist_mm = (float)raw_left_dist;
			} else {
				wall_sensor_left_hist[wall_sensor_hist_idx] = (uint16_t)last_known_left_dist_mm;
			}

			// Increment index for the circular buffer.
			wall_sensor_hist_idx = (wall_sensor_hist_idx + 1) % WALL_SENSOR_FILTER_SIZE;

			// Calculate the average of the last N readings.
			uint32_t sum_right = 0;
			uint32_t sum_left = 0;
			for (int i = 0; i < WALL_SENSOR_FILTER_SIZE; ++i) {
				sum_right += wall_sensor_right_hist[i];
				sum_left += wall_sensor_left_hist[i];
			}
			
			float right_dist_avg = (float)sum_right / WALL_SENSOR_FILTER_SIZE;
			float left_dist_avg  = (float)sum_left / WALL_SENSOR_FILTER_SIZE;
			// --- Unified PID Calculation ---
			// The error is the difference between the stabilized, filtered distances.
			// The setpoint is 0, aiming for perfect centering.
			wall_pid_pv = right_dist_avg - left_dist_avg;
			wall_pid_sp = 0.0;
			Pid_wall->Compute();
			wall_correction = wall_pid_cv;
			
		}

		// 5. Combine the outputs.
		// The master distance PID output (pid_l_cv) determines the base speed.
		// The wall follower PID output (wall_correction) provides steering.

		// Feed-forward for the desired speed.
		// The 'bias' variable acts as a feed-forward gain (Kff).
		// It should be tuned to convert speed (mm/s) to a rough PWM percentage.
		// A good starting value might be around 0.1 (e.g., 250 mm/s * 0.1 = 25% PWM).
		// This 'ff' term is the main driver of speed, while the PID corrects for position errors.
		double ff = bias * current_speed;
		double base_speed_cv = pid_l_cv + ff;

		// A positive wall_correction steers LEFT (increase left speed, decrease right).
		double cv_left = base_speed_cv + wall_correction;
		double cv_right = base_speed_cv - wall_correction;
		// Update pwm duty % and select forward or reverse direction for the motor driver:
		set_pwm_duty(cv_right, cv_left);
		_dir_fwd_l();
		_dir_fwd_r();
	}
}

/*
	Function: reset_pid
	--------------------
	Reset internal values and input-outputs for selected PID controller

	dir: select which PID controller (LEFT, RIGTH or BOTH)
 */
void MotorDrive::reset_pid(int dir)
{
	if(dir == LEFT or dir == BOTH)
	{
		final_pos_l = 0;
		angle_pv = 0;
		angle_sp = 0;
		angle_cv = 0;
		pid_l_pv = 0;
		pid_l_sp = 0;
		pid_l_cv = 0;
		Pid_l->Initialize();
	}
	if(dir == RIGHT or dir == BOTH)
	{
		final_pos_r = 0;
		pid_r_pv = 0;
		pid_r_sp = 0;
		pid_r_cv = 0;
		Pid_r->Initialize();
	}
}

/*
	Function: _dir_fwd_l
	--------------------
	Sets IN1/IN2 pins to move left motor forward
 */
void MotorDrive::_dir_fwd_l()
{
	HAL_GPIO_WritePin(in1_l_port, in1_l_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(in2_l_port, in2_l_pin, GPIO_PIN_SET);
}
/*
	Function: _dir_rev_l
	--------------------
	Sets IN1/IN2 pins to move left motor reverse
 */
void MotorDrive::_dir_rev_l()
{
	HAL_GPIO_WritePin(in1_l_port, in1_l_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(in2_l_port, in2_l_pin, GPIO_PIN_RESET);
}
/*
	Function: _dir_fwd_r
	--------------------
	Sets IN1/IN2 pins to move right motor forward
 */
void MotorDrive::_dir_fwd_r()
{
	HAL_GPIO_WritePin(in1_r_port, in1_r_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(in2_r_port, in2_r_pin, GPIO_PIN_RESET);
}
/*
	Function: _dir_rev_r
	--------------------
	Sets IN1/IN2 pins to move right motor reverse
 */
void MotorDrive::_dir_rev_r()
{
	HAL_GPIO_WritePin(in1_r_port, in1_r_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(in2_r_port, in2_r_pin, GPIO_PIN_SET);
}

/*
	Function: enc_l_inc
	--------------------
	Increse PV value according to encoder resolution
 */
void MotorDrive::enc_l_inc()
{
	pid_l_pv += mm_per_pulse;
}
/*
	Function: enc_l_inc
	--------------------
	Decrease PV value according to encoder resolution
 */
void MotorDrive::enc_l_dec()
{
	pid_l_pv -= mm_per_pulse;
}
/*
	Function: enc_l_inc
	--------------------
	Increse PV value according to encoder resolution
 */
void MotorDrive::enc_r_inc()
{
	pid_r_pv += mm_per_pulse;
}
/*
	Function: enc_l_inc
	--------------------
	Decrease PV value according to encoder resolution
 */
void MotorDrive::enc_r_dec()
{
	pid_r_pv -= mm_per_pulse;
}

PidData_t MotorDrive::get_pid_data()
{
	struct PidData dummy;

	dummy.pv_l = pid_l_pv;
	dummy.sp_l = pid_l_sp;
	dummy.cv_l = pid_l_cv;
	dummy.pv_r = pid_r_pv;
	dummy.sp_r = pid_r_sp;
	dummy.cv_r = pid_r_cv;
	return dummy;
}
