/*
 * preset_matricies.h
 *
 *  Created on: Apr 15, 2021
 *      Author: BAREND JACOBUS Edwards
 */

#ifndef INC_MATRIX_H_
#define INC_MATRIX_H_

// Includes
#include "main.h"


// Defines


// Function Prototypes
void light_up_LED(uint8_t x, uint8_t y);
void reset_all_ports(uint8_t isBallSkipped);
void copy_to_display(uint8_t buffer[][8]);
void load_preset_maze(uint8_t maze_num);
void display_matrix();
void increment_column();
uint8_t checkMazeBallMove(int x_increment, int y_increment);
uint8_t checkBatMove(int x_increment, int y_increment);
void move_tennis_ball();
void reset_debug_leds();
void light_debug_led(int led_num, uint8_t isReset);
void load_preset_num_to_display(uint8_t preset);
void update_imu_direction(int imu_x, int imu_y);
void move_bat_imu();
void move_ball_maze_imu();

// Variables
enum Status {
	CALIBRATION,
	CORNERS,
	MAZE_SELECTION,
	MAZE,
	TENNIS,
	BALL
};






#endif /* INC_MATRIX_H_ */
