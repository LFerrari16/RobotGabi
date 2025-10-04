/*
 * navigation.cpp
 *
 *  Created on: Sep 20, 2025
 *      Author: Leito.Ferr
 *  This class provides high-level navigation and maze-solving logic,
 *  ported from the Python version of maze_navigation.py.
 */


#include "navigation.h"
#include "main.h" // For HAL_Delay
// Debug only
#include "usart.h" // For debug output via UART_BT
#include <stdio.h> // For sprintf

#define MAZE_UNVISITED 0xFFFF
#define MAZE_DEAD_END  0xFFFE

#define WALL_NORTH (1 << 0)
#define WALL_EAST  (1 << 1)
#define WALL_SOUTH (1 << 2)
#define WALL_WEST  (1 << 3)

/**
 * @brief Constructor for the Navigation class.
 * @param motor Pointer to the MotorDrive object for low-level control.
 * @param sensors Pointer to the VL53L0X_array object for distance sensing.
 */
Navigation::Navigation(MotorDrive* motor, VL53L0X_array* sensors) {
    p_motor_drive = motor;
    p_tof_sensors = sensors;

    // Start at the bottom-left corner of the maze grid.
    x_pos = 0;
    y_pos = MAZE_HEIGHT - 1;
    orientation = NORTH;

    robot_state = IDLE;
    solving_rule = RULE_RIGHT_HAND; // Default rule

    reset_for_explore();
}

/**
 * @brief Moves the robot forward by one cell distance.
 *        This is a blocking call. It also checks for a front wall before moving.
 */
void Navigation::move_forward_one_cell() {
    // Check for a wall in front before moving to replicate original functionality
    Walls current_walls = get_walls();
    if (current_walls.front) {
        // There is a wall in front, do not move.
        // Optionally, add some feedback like a buzzer or LED.
        return;
    }

     // Enable wall following logic inside MotorDrive
    p_motor_drive->set_wall_following(true);

    // Execute the move. The MotorDrive class will handle the wall following internally.
    p_motor_drive->move_forward(CELL_WIDTH_MM, 5000); // 5-second timeout

    // After moving, update the robot's coordinates in the grid
    update_position();
}

/**
 * @brief Turns the robot 90 degrees to the right.
 *        This is a blocking call.
 */
void Navigation::turn_right() {
    p_motor_drive->turn(-86, 3000); // 90 degrees, 3-second timeout
    orientation = (Orientation)((orientation + 1) % 4);
}

/**
 * @brief Turns the robot 90 degrees to the left.
 *        This is a blocking call.
 */
void Navigation::turn_left() {
    p_motor_drive->turn(86, 3000); // -90 degrees, 3-second timeout
    // Handle wrap-around for enum
    if (orientation == NORTH) {
        orientation = WEST;
    } else {
        orientation = (Orientation)(orientation - 1);
    }
}

/**
 * @brief Turns the robot 180 degrees.
 *        This is a blocking call.
 */
void Navigation::turn_around() {
    p_motor_drive->turn(172, 4000); // 180 degrees, 4-second timeout
    orientation = (Orientation)((orientation + 2) % 4);
}

/**
 * @brief Reads the distance sensors to detect walls.
 * @return A Walls struct indicating the presence of front, left, and right walls.
 */
Walls Navigation::get_walls() {
    Walls walls;

    // Update sensor readings
    p_tof_sensors->ping();

    // Index 0: Right sensor (tof_R)
    // Index 1: Front-Right sensor (tof_FR)
    // Index 2: Front-Left sensor (tof_FL)
    // Index 3: Left sensor (tof_L)
    // The user must ensure VL53L0X_array is initialized with 4 sensors in this order.

    uint16_t right_dist  = p_tof_sensors->get_distance(0);
    uint16_t frontR_dist = p_tof_sensors->get_distance(1);
    uint16_t frontL_dist = p_tof_sensors->get_distance(2);
    uint16_t left_dist   = p_tof_sensors->get_distance(3);

    // Average the two front sensors for front wall detection
    uint16_t front_dist = (frontR_dist + frontL_dist) / 2;

    walls.front = front_dist < FRONT_DIST_LIMIT;
    walls.left  = left_dist  < WALL_THRESHOLD_MM;
    walls.right = right_dist < WALL_THRESHOLD_MM;

    return walls;
}

/**
 * @brief Updates the robot's (x, y) coordinates based on its orientation after a forward move.
 */
void Navigation::update_position() {
    switch (orientation) {
        case NORTH:
            y_pos--; // Moving North decreases the Y-index in array coordinates
            break;
        case EAST:
            x_pos++;
            break;
        case SOUTH:
            y_pos++; // Moving South increases the Y-index in array coordinates
            break;
        case WEST:
            x_pos--;
            break;
    }
}

/**
 * @brief Gets the current position and orientation of the robot.
 * @param x Reference to store the x-coordinate.
 * @param y Reference to store the y-coordinate.
 * @param o Reference to store the orientation.
 */
void Navigation::get_position(int& x, int& y, Orientation& o) {
    x = x_pos;
    y = y_pos;
    o = orientation;
}

RobotState Navigation::get_robot_state() {
    return robot_state;
}

/**
 * @brief Checks if the robot has reached the goal.
 * @return True if the goal is detected, false otherwise.
 */
bool Navigation::is_goal() {
    // The floor sensor is assumed to be on a white line, which means it will reflect IR,
    // so the pin will read LOW (GPIO_PIN_RESET) when on the goal.
    // The user must verify the sensor's logic.
    // if (HAL_GPIO_ReadPin(FLOOR_SENS_GPIO_Port, FLOOR_SENS_Pin) == GPIO_PIN_RESET) {
    //     robot_state = PLANNING;
    //     return true;
    // }
    return false;
}

/**
 * @brief Resets the maze map and robot state for a new exploration run.
 */
void Navigation::reset_for_explore() {
    // Start at the bottom-left corner of the maze grid.
    x_pos = 0;
    y_pos = MAZE_HEIGHT - 1;
    orientation = NORTH;
    maze_counter = 1;
    robot_state = IDLE;

    for (int y = 0; y < MAZE_HEIGHT; ++y) {
        for (int x = 0; x < MAZE_WIDTH; ++x) {
            maze_dist[y][x] = MAZE_UNVISITED;
            maze_walls[y][x] = 0;
        }
    }
}

/**
 * @brief Sets the maze solving algorithm to be used.
 * @param rule The rule to use (RULE_LEFT_HAND or RULE_RIGHT_HAND).
 */
void Navigation::set_solving_rule(MazeRule rule) {
    this->solving_rule = rule;
    this->robot_state = EXPLORING;
}

/**
 * @brief Updates the internal wall map based on current sensor readings.
 */
void Navigation::_update_maze_walls(const Walls& walls) {
    uint8_t current_walls = 0;
    if (walls.front) {
        if (orientation == NORTH) current_walls |= WALL_NORTH;
        if (orientation == EAST)  current_walls |= WALL_EAST;
        if (orientation == SOUTH) current_walls |= WALL_SOUTH;
        if (orientation == WEST)  current_walls |= WALL_WEST;
    }
    if (walls.right) {
        if (orientation == NORTH) current_walls |= WALL_EAST;
        if (orientation == EAST)  current_walls |= WALL_SOUTH;
        if (orientation == SOUTH) current_walls |= WALL_WEST;
        if (orientation == WEST)  current_walls |= WALL_NORTH;
    }
    if (walls.left) {
        if (orientation == NORTH) current_walls |= WALL_WEST;
        if (orientation == EAST)  current_walls |= WALL_NORTH;
        if (orientation == SOUTH) current_walls |= WALL_EAST;
        if (orientation == WEST)  current_walls |= WALL_SOUTH;
    }
    maze_walls[y_pos][x_pos] |= current_walls;

    // Also update the neighbor's wall
    if (walls.front) {
        if ((orientation == NORTH) && (y_pos > 0)) maze_walls[y_pos - 1][x_pos] |= WALL_SOUTH;
        if ((orientation == EAST) && (x_pos < MAZE_WIDTH - 1)) maze_walls[y_pos][x_pos + 1] |= WALL_WEST;
        if ((orientation == SOUTH) && (y_pos < MAZE_HEIGHT - 1)) maze_walls[y_pos + 1][x_pos] |= WALL_NORTH;
        if ((orientation == WEST) && (x_pos > 0)) maze_walls[y_pos][x_pos - 1] |= WALL_EAST;
    }
}

/**
 * @brief Turns and moves to an adjacent cell.
 */
void Navigation::_move_to_neighbor(int target_x, int target_y) {
    int dx = target_x - x_pos;
    int dy = target_y - y_pos;

    Orientation target_orientation;
    if (dx == 1) target_orientation = EAST;
    else if (dx == -1) target_orientation = WEST;
    else if (dy == 1) target_orientation = SOUTH; // Inverted Y-axis for array
    else if (dy == -1) target_orientation = NORTH;
    else return; // Not a neighbor

    int orientation_diff = (target_orientation - orientation + 4) % 4;

    if (orientation_diff == 1) turn_right();
    else if (orientation_diff == 2) turn_around();
    else if (orientation_diff == 3) turn_left();

    move_forward_one_cell();
}

/**
 * @brief Executes one step of a maze-solving algorithm (e.g., left-hand rule).
 *        This function contains the high-level logic to decide the next move.
 */
void Navigation::explore_step() {
    // --- Debug output ---
    // This will send the robot's current state over Bluetooth.
    // Make sure your terminal is connected to see these messages.
    char dbg_buf[128];
    int len;
    len = sprintf(dbg_buf, "Pos:(%d,%d), O:%d | ", x_pos, y_pos, orientation);
    HAL_UART_Transmit(&UART_BT, (uint8_t*)dbg_buf, len, 100);
    
    // This function implements the DFS-like exploration from maze_solving.py
    Walls walls = get_walls();
        _update_maze_walls(walls);

    // --- Forward Move Logic: Find an unvisited neighbor ---
    // The order of checking determines the "hand on wall" rule for breaking ties.
    Orientation check_order[4];
    if (solving_rule == RULE_RIGHT_HAND) {
        check_order[0] = (Orientation)((orientation + 0) % 4); // Forward
        check_order[1] = (Orientation)((orientation + 1) % 4); // Right
        check_order[2] = (Orientation)((orientation + 3) % 4); // Left
        check_order[3] = (Orientation)((orientation + 2) % 4); // Back
    } else { // Left Hand Rule
        check_order[0] = (Orientation)((orientation + 0) % 4); // Forward
        check_order[1] = (Orientation)((orientation + 3) % 4); // Left
        check_order[2] = (Orientation)((orientation + 1) % 4); // Right
        check_order[3] = (Orientation)((orientation + 2) % 4); // Back
    }

    for (int i = 0; i < 4; ++i) {
        Orientation dir = check_order[i];
        int next_x = x_pos, next_y = y_pos;
        uint8_t wall_to_check = 0;

        if (dir == NORTH) { next_y--; wall_to_check = WALL_NORTH; }
        if (dir == EAST)  { next_x++; wall_to_check = WALL_EAST;  }
        if (dir == SOUTH) { next_y++; wall_to_check = WALL_SOUTH; }
        if (dir == WEST)  { next_x--; wall_to_check = WALL_WEST;  }

        // Check if neighbor is within bounds and there's no wall
        if (next_x >= 0 && next_x < MAZE_WIDTH && next_y >= 0 && next_y < MAZE_HEIGHT &&
            !(maze_walls[y_pos][x_pos] & wall_to_check)) {

            if (maze_dist[next_y][next_x] == MAZE_UNVISITED) {
                // Found an unvisited cell, move to it
                len = sprintf(dbg_buf, "Unvisited neighbor. Moving.\r\n");
                HAL_UART_Transmit(&UART_BT, (uint8_t*)dbg_buf, len, 100);

                maze_dist[y_pos][x_pos] = maze_counter++;
                _move_to_neighbor(next_x, next_y);
                return;
            }
        }
    }

    // --- Backtrack Logic: No unvisited neighbors found ---
    len = sprintf(dbg_buf, "Dead end. Backtracking.\r\n");
    HAL_UART_Transmit(&UART_BT, (uint8_t*)dbg_buf, len, 100);

    for (int i = 0; i < 4; ++i) {
        Orientation dir = (Orientation)i;
        int next_x = x_pos, next_y = y_pos;
        uint8_t wall_to_check = 0;

        if (dir == NORTH) { next_y--; wall_to_check = WALL_NORTH; }
        if (dir == EAST)  { next_x++; wall_to_check = WALL_EAST;  }
        if (dir == SOUTH) { next_y++; wall_to_check = WALL_SOUTH; }
        if (dir == WEST)  { next_x--; wall_to_check = WALL_WEST;  }

        if (next_x >= 0 && next_x < MAZE_WIDTH && next_y >= 0 && next_y < MAZE_HEIGHT &&
            !(maze_walls[y_pos][x_pos] & wall_to_check)) {

            if (maze_dist[next_y][next_x] == maze_counter - 1) {
                // This is the cell we came from, backtrack to it
                maze_dist[y_pos][x_pos] = MAZE_DEAD_END;
                maze_counter--;
                _move_to_neighbor(next_x, next_y);
                return;
            }
        }
    }
}

void Navigation::plan_fast_run() {
    // This is a simplified version of maze_array_decode
    // It generates a string of commands: F (forward), L (left), R (right)
    // It does not yet consolidate multiple 'F' moves, but it's a start.

    if (robot_state != PLANNING) return;

    int path_x = x_pos;
    int path_y = y_pos;
    uint16_t current_dist = maze_dist[path_y][path_x];
    char reverse_path[MAZE_WIDTH * MAZE_HEIGHT] = {0};
    int path_idx = 0;

    // Trace back from goal to start
    while (current_dist > 1) {
        uint16_t next_dist = current_dist - 1;
        // Find neighbor with distance == next_dist
        // North
        if (path_y > 0 && !(maze_walls[path_y][path_x] & WALL_NORTH) && maze_dist[path_y - 1][path_x] == next_dist) {
            reverse_path[path_idx++] = 'N'; path_y--;
        } // East
        else if (path_x < MAZE_WIDTH - 1 && !(maze_walls[path_y][path_x] & WALL_EAST) && maze_dist[path_y][path_x + 1] == next_dist) {
            reverse_path[path_idx++] = 'E'; path_x++;
        } // South
        else if (path_y < MAZE_HEIGHT - 1 && !(maze_walls[path_y][path_x] & WALL_SOUTH) && maze_dist[path_y + 1][path_x] == next_dist) {
            reverse_path[path_idx++] = 'S'; path_y++;
        } // West
        else if (path_x > 0 && !(maze_walls[path_y][path_x] & WALL_WEST) && maze_dist[path_y][path_x - 1] == next_dist) {
            reverse_path[path_idx++] = 'W'; path_x--;
        }
        current_dist = next_dist;
    }

    // Convert absolute path (N,E,S,W) to relative robot commands (F,L,R)
    Orientation current_orientation = NORTH;
    fast_path_len = 0;
    for (int i = path_idx - 1; i >= 0; i--) { // Iterate backwards to get correct path
        Orientation target_orientation;
        if (reverse_path[i] == 'N') target_orientation = NORTH;
        if (reverse_path[i] == 'E') target_orientation = EAST;
        if (reverse_path[i] == 'S') target_orientation = SOUTH;
        if (reverse_path[i] == 'W') target_orientation = WEST;

        int diff = (target_orientation - current_orientation + 4) % 4;
        if (diff == 1) fast_path[fast_path_len++] = 'R';
        if (diff == 2) { fast_path[fast_path_len++] = 'R'; fast_path[fast_path_len++] = 'R'; } // Turn around
        if (diff == 3) fast_path[fast_path_len++] = 'L';

        fast_path[fast_path_len++] = 'F';
        current_orientation = target_orientation;
    }
    fast_path[fast_path_len] = '\0';

    robot_state = AWAITING_FAST_RUN;
}

void Navigation::execute_fast_run() {
    if (robot_state != FAST_RUN) return;

    // This is a blocking function that executes the whole path
    for (int i = 0; i < fast_path_len; ++i) {
        char command = fast_path[i];
        if (command == 'F') {
            move_forward_one_cell();
        } else if (command == 'L') {
            turn_left();
        } else if (command == 'R') {
            turn_right();
        }
    }
    robot_state = FINISHED;
}