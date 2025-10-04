/*
 * navigation.h
 *
 * Created on: Sep 20, 2025
 * Author: Leito.Ferr
 */

#ifndef INC_NAVIGATION_H_
#define INC_NAVIGATION_H_

#include "motor_drive.h"
#include "VL6180X_Array.h" // <-- CAMBIO

// Constants adapted from Python code
#define CELL_WIDTH_MM           248.0f
#define FRONT_DIST_LIMIT        80.0f // Wall detection threshold for front sensor in mm
#define WALL_THRESHOLD_MM       150.0f // Wall detection threshold for side sensors in mm

// Maze dimensions (example, user can change)
#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16

// Robot orientation
typedef enum {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
} Orientation;

// High-level robot state machine
typedef enum {
    IDLE,
    EXPLORING,
    PLANNING,
    AWAITING_FAST_RUN,
    FAST_RUN,
    FINISHED
} RobotState;

// Maze solving algorithm rule
typedef enum {
    RULE_LEFT_HAND,
    RULE_RIGHT_HAND
} MazeRule;

// Struct to hold wall information
struct Walls {
    bool front;
    bool left;
    bool right;
};

class Navigation {
public:
    // Constructor
    Navigation(MotorDrive* motor, VL6180X_Array* sensors); // <-- CAMBIO

    // High-level movement commands
    void move_forward_one_cell();
    void turn_right();
    void turn_left();
    void turn_around();

    // Sensor and state functions
    Walls get_walls();
    void update_position();
    void get_position(int& x, int& y, Orientation& o);
    bool is_goal();
    RobotState get_robot_state();

    // Maze solving logic
    void reset_for_explore();
    void explore_step();
    void plan_fast_run();
    void execute_fast_run();
    void set_solving_rule(MazeRule rule);

    // Public member for main loop state changes
    RobotState robot_state;

private:
    MotorDrive* p_motor_drive;
    VL6180X_Array* p_tof_sensors; // <-- CAMBIO

    MazeRule   solving_rule;

    // Robot's state in the maze
    int x_pos;
    int y_pos;
    Orientation orientation;
    
    // Maze mapping data
    uint8_t maze_walls[MAZE_HEIGHT][MAZE_WIDTH]; // Bitmask for walls (N, E, S, W)
    uint16_t maze_dist[MAZE_HEIGHT][MAZE_WIDTH];   // Distance from start for pathfinding
    uint16_t maze_counter;

    // Optimized path for fast run
    char fast_path[MAZE_WIDTH * MAZE_HEIGHT];
    int fast_path_len;

    // Private helper methods
    void _update_maze_walls(const Walls& walls);
    void _move_to_neighbor(int target_x, int target_y);
    // Variables para estabilizar el seguimiento de muros en intersecciones
    float last_left_dist_mm;
    float last_right_dist_mm;
};

#endif /* INC_NAVIGATION_H_ */