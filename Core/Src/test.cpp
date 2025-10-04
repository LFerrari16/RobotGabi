/*
 * test.cpp
 *
 *  Created on: Mar 17, 2024
 *      Author: Marto
 */

#include "test.h"
#define MSG_LENGTH	10

// Lightweight atoi to avoid pulling in heavy stdlib functions like atof
int my_atoi(const char *str)
{
	int res = 0;
	int sign = 1;
	int i = 0;

	// If number is negative, then update sign
	if (str[0] == '-')
	{
		sign = -1;
		i++;
	}

	// Iterate through all digits and update the result
	for (; str[i] != '\0' && str[i] >= '0' && str[i] <= '9'; ++i)
	{
		res = res * 10 + str[i] - '0';
	}

	return sign * res;
}

static bool compare_arrays(const uint8_t* str1, const uint8_t* str2)
{
	// Use strncmp for a safer 2-character comparison.
	// It's generally better than a manual loop.
	return (strncmp((const char*)str1, (const char*)str2, 2) == 0);
}

int get_param_value(uint8_t* str)
{
	int i=0, j=3;
	char cropped_str[10] = {0}; // Initialize to zeros

	// Loop with a check on 'i' to prevent buffer overflow on cropped_str.
	// Using '&&' is the standard C++ operator for 'and'.
	// The command string is null-terminated ('\0'), not terminated by carriage return ('\r').
	while((str[j] != '\0') && (j < MSG_LENGTH) && (i < (sizeof(cropped_str) - 1)))
	{
		cropped_str[i] = str[j];
		i++;
		j++;
	}
	cropped_str[i]='\0';
	return my_atoi(cropped_str);
}

BTcommander::BTcommander(UART_HandleTypeDef* uart_p)
{
	uart = uart_p;
}

struct bt_command BTcommander::get_bt_command()
{
	uint8_t str[MSG_LENGTH] = {0};
	uint8_t received_char;
	int i = 0;
	HAL_StatusTypeDef status;
	struct bt_command dummy;

	// This function is now mostly non-blocking.
	// It reads one character at a time until a full command is received (ending in '\r' or '\n')
	// or the buffer is full.
	while (i < (MSG_LENGTH - 1))
	{
		// Use a short timeout to check for a character.
		// If no character is received, we just exit and try again on the next loop in main.
		status = HAL_UART_Receive(uart, &received_char, 1, 10);
		if (status == HAL_OK)
			{
				// If we receive a terminator character, the command is complete.
				if ((received_char == '\r') || (received_char == '\n'))
				{
					if (i > 0) // We have a command string to process
					{
						break;
					}
					// Otherwise, it's an empty line, just ignore and continue waiting.
				}
				else
				{
					// Add the character to our buffer.
					str[i++] = received_char;
				}
			}
			else // HAL_TIMEOUT or HAL_ERROR
			{
				// If we timed out waiting for a character, it means no command is being sent.
				// Return an invalid command and let the main loop continue.
				dummy.name = -1;
				dummy.param = 0;
				return dummy;
			}
	}
	str[i] = '\0'; // Null-terminate the string.

	// If we have a command string (i > 0), process it.
	if (i > 0)
	
	{
		for(int j=0; j<COMM_NBR; j++)
		{
			if(compare_arrays(str, commands[j]))
			{
				dummy.name = j;
				dummy.param = get_param_value(str);
				return dummy;
			}
		}
	}

	dummy.name = -1;
	dummy.param = 0;
	return dummy;
}


