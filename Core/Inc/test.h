/*
 * test.h
 *
 *  Created on: Mar 17, 2024
 *      Author: Marto
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_

#include "usart.h"
#include "motor_drive.h"
#include <string.h> // For strncmp

#define COMM_NBR	17
#define	MOVE_FWD	0
#define	MOVE_REV	1
#define	SET_KP		2
#define	SET_KI		3
#define	SET_KD		4
#define	SET_SPEED	5
#define	SET_ACCEL	6
#define	TURN		7
#define SET_WALL_KP 8
#define SET_WALL_KI 9
#define SET_WALL_KD 10
#define SENSOR_STATUS 11
#define SET_ANGLE_KP 12
#define SET_ANGLE_KI 13
#define SET_ANGLE_KD 14
#define	MOVE_CELLS	 15
#define	SET_BIAS	 16

struct bt_command
{
	int name;
	int param;
};

class BTcommander
{
	private:
		UART_HandleTypeDef *uart;
		uint8_t commands[COMM_NBR][2]=
		{
				{'m', 'f'},		// Move forward
				{'m', 'r'},		// Move reverse
				{'k', 'p'},		// Set Kp
				{'k', 'i'},		// Set Ki
				{'k', 'd'},		// Set Kd
				{'s', 'p'},		// Set speed
				{'s', 'a'},		// Set acceleration
				{'t', 'u'},		// TURN
				{'w', 'k'},		// Set Wall Kp
				{'w', 'i'},		// Set Wall Ki
				{'w', 'd'},		// Set Wall Kd
				{'s', 's'},		// Sensor Status
				{'a', 'k'},		// Set Angle Kp
				{'a', 'i'},		// Set Angle Ki
				{'a', 'd'},		// Set Angle Kd
				{'m', 'c'},		// Move cells
				{'s', 'b'}		// Set FF Bias

		};
	public:
		BTcommander(UART_HandleTypeDef*);
		struct bt_command get_bt_command();
};

#endif /* INC_TEST_H_ */
