/*
 * main_cpp.cpp
 *
 *  Created on: Feb 22, 2024
 *      Author: Marto
 */

#include <stdio.h>
#include "main_cpp.h"
#include "main.h"
#include "motor_drive.h"
#include "test.h"
#include "navigation.h"
#include "VL6180X_Array.h"

extern "C" {
#include "i2c.h"
#include "usart.h"
#include "tim.h"
}

VL6180X_Array* mySensors;

float angle = 0;
uint8_t Message[64];
uint8_t MessageLen;

MotorDrive *myMotor;
Navigation *myNavigation;
BTcommander *myBT;

// Angle calculation timer interrupt handler:
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	#ifdef DEBUG
	unsigned long t1 = DWT->CYCCNT;	// Get cycle counter ticks:
	#endif

	// MessageLen = sprintf((char*)Message, "angle=%d\r\n", (int)angle);
	// HAL_UART_Transmit(&huart2, Message, MessageLen, 100); // Received an ACK at that address

//MotorDrive myMotor;

/*
	Function: HAL_TIM_PeriodElapsedCallback
	--------------------
	PID timer interrupt handler. Calls PID update method every "PID_SAMPLE_TIME_MS" milliseconds

	htim: standard param for timer interrupt, identifies which timer triggered the INT
 */
	myMotor->update_pid();

	#if DEBUG_TEST
	// Every 100ms, send PID info via BT:
	static int count=0;
	if(count>=50/PID_SAMPLE_TIME_MS)
	{
		char str[50];
		int len;
		PidData_t dummy = myMotor->get_pid_data();
		len = sprintf(str, "$%d %d %d %d %d %d;",
				(int)dummy.pv_l, (int)dummy.sp_l, (int)dummy.cv_l,
				(int)dummy.pv_r, (int)dummy.sp_r, (int)dummy.cv_r);
		//if(len>=50)
		//	len = sprintf(str, "Length error!\n");
		HAL_UART_Transmit(&UART_BT, (uint8_t*)str, len, 100);
		count=0;
	}
	else
		count++;
	#endif

	#ifdef DEBUG
	unsigned long t2 = DWT->CYCCNT;
	float diff __attribute__((unused)) = (t2-t1)/72;	// Calc interrupt time length in usec. (fclk=72MHz)
	#endif
}

/*
	Function: HAL_GPIO_EXTI_Callback
	--------------------
	GPIO interrupt handler. Updates encoder pulse counters

	GPIO_Pin: standard param for gpio interrupt, identifies which input triggered the INT
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	#ifdef DEBUG
	unsigned long t1 = DWT->CYCCNT;	// Get cycle counter ticks:
	#endif

	GPIO_PinState val;

	// Checks which channel triggered the INT (set to trigger by both A and B signals):
	if(GPIO_Pin == ENC_L_A_Pin)
	{
		// Checks B channel to know which direction the encoder is going:
		// (INT is triggered with input both rising and falling)
		val = HAL_GPIO_ReadPin(ENC_L_A_GPIO_Port, ENC_L_A_Pin);
		if(HAL_GPIO_ReadPin(ENC_L_B_GPIO_Port, ENC_L_B_Pin) != val)
			myMotor->enc_l_dec();
		else
			myMotor->enc_l_inc();
	}
	if(GPIO_Pin == ENC_L_B_Pin)
	{
		val = HAL_GPIO_ReadPin(ENC_L_B_GPIO_Port, ENC_L_B_Pin);
		if(HAL_GPIO_ReadPin(ENC_L_A_GPIO_Port, ENC_L_A_Pin) != val)
			myMotor->enc_l_inc();
		else
			myMotor->enc_l_dec();
	}

	// Same for right motor:
	if(GPIO_Pin == ENC_R_A_Pin)
	{
		val = HAL_GPIO_ReadPin(ENC_R_A_GPIO_Port, ENC_R_A_Pin);
		if(HAL_GPIO_ReadPin(ENC_R_B_GPIO_Port, ENC_R_B_Pin) != val)
			myMotor->enc_r_inc();
		else
			myMotor->enc_r_dec();
	}
	if(GPIO_Pin == ENC_R_B_Pin)
	{
		val = HAL_GPIO_ReadPin(ENC_R_B_GPIO_Port, ENC_R_B_Pin);
		if(HAL_GPIO_ReadPin(ENC_R_A_GPIO_Port, ENC_R_A_Pin) != val)
			myMotor->enc_r_dec();
		else
			myMotor->enc_r_inc();
	}

	#ifdef DEBUG
	unsigned long t2 = DWT->CYCCNT;
	float diff __attribute__((unused)) = (t2-t1)/72;	// Calc interrupt time length in usec. (fclk=72MHz)
	#endif
}

/*
	Function: main_cpp
	--------------------
	Main program routine, should never return
 */
int main_cpp()
{
	float mm_per_rev = WHEEL_DIAM_MM *3.1416;
	struct bt_command command;

	// Instantiate sensor array.
	// NOTE: This assumes 4 sensors are configured in this order:
	// 0:Right, 1:Front-Right, 2:Front-Left, 3:Left
	mySensors = new VL6180X_Array(&I2C_SENSORS, 4);

	// Class setup. All parameters defined in Device config tool -> User constants/Pin labels:
	// main_cpp.cpp (Línea Corregida)
	myMotor = new MotorDrive(
    &htim3, TIM_CHANNEL_1,            // Motor Izq: TIM3, Canal 1
    IN1_L_GPIO_Port, IN1_L_Pin,
    &htim3, TIM_CHANNEL_3,            // Motor Der: TIM3, Canal 3
    IN1_R_GPIO_Port, IN1_R_Pin,
    INIT_SPEED, INIT_ACCEL, mySensors);
	myMotor->set_sp(0,0);

	myNavigation = new Navigation(myMotor, mySensors);

	myBT = new BTcommander(&UART_BT);

	PID_Timer.Init.Period = PID_SAMPLE_TIME_MS*1000-1;	// Set PID timer period
	HAL_TIM_Base_Start_IT(&PID_Timer);	// Start timer interrupts

  	#ifdef DEBUG
  	// Enable cycle counter to be able measure clock ticks:
  	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  	DWT->CYCCNT = 0;
  	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  	#endif
 	
	// Wait for user button press to start the maze solving sequence.
	// NOTE: Assumes USER_BTN_GPIO_Port and USER_BTN_Pin are defined from CubeMX.
	// The button is assumed to be Active Low (press to ground).
	// while(HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_SET)
	// {
	// 	HAL_Delay(10);
	// }

#if DEBUG_TEST
	// --- Test and Tuning Mode ---
	// This loop waits for Bluetooth commands to test motors and tune PIDs.
	// It is active only when DEBUG_TEST is defined as 1 in main.h.
	// Note: For wall following tests, you must enable it manually via a command if needed,
	// or modify a command like MOVE_FWD to enable it before moving.
	while(1)
	{
		command = myBT->get_bt_command();
		if (command.name != -1) // Check if a valid command was received
		{
			switch(command.name)
			{
				case TURN:
					myMotor->turn(command.param, 5000);
					break;
				case MOVE_FWD:
					// Parameter is number of cells. Enable wall following for this test move.
					myMotor->set_wall_following(false); // Disable wall following for simple test
					myMotor->move_forward(command.param * mm_per_rev, 5000);
					break;
				case MOVE_REV:
					// Parameter is number of cells. Enable wall following for this test move.
					myMotor->set_wall_following(true); // Disable wall following for simple test
					myMotor->move_forward(command.param * mm_per_rev, 5000);
					break;
				case SET_KP:
					myMotor->set_kp((double)command.param / 1000.0);
					break;
				case SET_KI:
					myMotor->set_ki((double)command.param / 1000.0);
					break;
				case SET_KD:
					myMotor->set_kd((double)command.param / 1000.0);
					break;
				case SET_SPEED:
					myMotor->set_speed(command.param);
					break;
				case SET_ACCEL:
					myMotor->set_accel(command.param);
					break;
				case SET_WALL_KP:
					myMotor->set_wall_kp((double)command.param / 1000.0);
					break;
				case SET_WALL_KI:
					myMotor->set_wall_ki((double)command.param / 1000.0);
					break;
				case SET_WALL_KD:
					myMotor->set_wall_kd((double)command.param / 1000.0);
					break;
				case SENSOR_STATUS:
				{
					// Read all sensors and report their values
					mySensors->ping();
					char sensor_str[64];
					int len = sprintf(sensor_str, "R:%d FR:%d FL:%d L:%d\r\n",
							mySensors->get_distance(0),
							mySensors->get_distance(1),
							mySensors->get_distance(2),
							mySensors->get_distance(3));
					HAL_UART_Transmit(&UART_BT, (uint8_t*)sensor_str, len, 100);
					break;
				}
				case SET_ANGLE_KP:
					myMotor->set_angle_kp((double)command.param / 1000.0);
					break;
				case SET_ANGLE_KI:
					myMotor->set_angle_ki((double)command.param / 1000.0);
					break;
				case SET_ANGLE_KD:
					myMotor->set_angle_kd((double)command.param / 1000.0);
					break;
				case MOVE_CELLS:
					// Parameter is number of cells. Wall following is ON for this command.
					myMotor->set_wall_following(true);
					myMotor->move_forward(command.param * CELL_WIDTH_MM, 5000);
					break;
				case SET_BIAS:
					myMotor->set_bias((double)command.param / 1000.0); // Divide by 1000 for fine control (e.g., send SB100 for bias=0.1)
					break;
				case MOTOR_TEST:
					myMotor->motor_test();
					break;
			}
		}
	}
	#else
	// --- Maze Solving Mode ---
	// This is the main application state machine for solving the maze.
	// It is active when DEBUG_TEST is not defined or is 0.
	while (1)
	{
		RobotState current_state = myNavigation->get_robot_state();

		switch (current_state) {
			case IDLE:
			case AWAITING_FAST_RUN:
			{
				// Wait for a button press to start either exploration or fast run.
				// This logic handles short vs. long press detection.
				// Assumes an Active-Low button (press to ground).

				// 1. Wait for the button to be released (in case it's stuck pressed).
				while(HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_RESET) {
					HAL_Delay(20);
				}

				// 2. Now, wait for a new press to start.
				while(HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_SET) {
					// Robot is waiting for input. A blinking LED could be useful here.
					HAL_Delay(20);
				}

				// 3. Button has been pressed. Debounce and record the start time.
				HAL_Delay(50); // Debounce delay
				uint32_t press_start_time = HAL_GetTick();
				bool long_press_indicated = false;

				// 4. Loop while the button is held down to measure press duration.
				while(HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_RESET) {
					// Indicate a long press with the LED after 3 seconds.
					if (!long_press_indicated && (HAL_GetTick() - press_start_time >= 3000)) {
						HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_SET); // Turn on LED for long press
						long_press_indicated = true;
					}
				}
				uint32_t press_duration = HAL_GetTick() - press_start_time;

				// 5. Button has been released. Turn off LED and debounce.
				HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
				HAL_Delay(50); // Debounce release

				// 6. Act based on the press duration.
				if (current_state == IDLE) { // First run
					myNavigation->reset_for_explore();
					if (press_duration >= 3000) {
						myNavigation->set_solving_rule(RULE_LEFT_HAND);
					} else {
						myNavigation->set_solving_rule(RULE_RIGHT_HAND);
					}
				} else { // Awaiting second run
					if (press_duration >= 3000) {
						// Long press starts the fast run
						myNavigation->robot_state = FAST_RUN;
					}
				}
				break;
			}

			case EXPLORING:
				mySensors->ping();
				if (myNavigation->is_goal()) {
					myMotor->stop_brake();
					// Goal found, transition to planning
				} else {
					myNavigation->explore_step();
				}
				break;

			case PLANNING:
				myNavigation->plan_fast_run();
				break;

			case FAST_RUN:
				myNavigation->execute_fast_run(); // This is a blocking call
				break;

			case FINISHED:
				// Do nothing, wait for reset or new command
				HAL_Delay(100);
				break;
		}	
	}
#endif
}
