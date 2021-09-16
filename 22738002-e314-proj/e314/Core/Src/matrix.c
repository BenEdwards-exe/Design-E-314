/*
 * matrix.c
 *
 *  Created on: Apr 15, 2021
 *      Author: BAREND JACOBUS Edwards
 */

// Includes
#include "matrix.h"
#include "main.h"
#include <math.h>


// Global Variables
volatile uint8_t ball_x_pos = 7;
volatile uint8_t ball_y_pos = 4;

volatile uint8_t bat_y_pos = 4;
volatile uint8_t bat_x_pos = 0;

volatile uint8_t display[8][8] = {0};

volatile enum Status matrixStatus;

volatile uint8_t column_to_display = 0;
volatile uint8_t display_matrix_ball_flag = 0; // 1 to display; 0 to not
volatile uint8_t display_matrix_end_flag = 0; // 1 to display; 0 to not
volatile uint8_t should_load_num_flag = 1;
volatile uint8_t should_load_maze_flag = 1;
volatile int selected_maze_num = 1;

volatile uint16_t tennis_ball_timeout_time = 700;
volatile uint8_t tennis_ball_hit_counter = 0;
volatile uint8_t tennis_ball_direction;
volatile uint8_t tennis_ball_velocity = 1;

volatile uint8_t imu_setup_flag = 1;
volatile int imu_direction_to_move[2] = {0};

extern uint32_t ball_maze_move_time;
extern uint32_t bat_move_time;

extern uint8_t maze[10];
extern uint8_t tennis[10];




static GPIO_TypeDef* row_ports[] = {
		row_0_GPIO_Port, row_1_GPIO_Port, row_2_GPIO_Port, row_3_GPIO_Port,
		row_4_GPIO_Port, row_5_GPIO_Port, row_6_GPIO_Port, row_7_GPIO_Port
};

static uint16_t row_pins[] = {
		row_0_Pin, row_1_Pin, row_2_Pin, row_3_Pin,
		row_4_Pin, row_5_Pin, row_6_Pin, row_7_Pin
};

static GPIO_TypeDef* col_ports[] = {
		col_0_GPIO_Port, col_1_GPIO_Port, col_2_GPIO_Port, col_3_GPIO_Port,
		col_4_GPIO_Port, col_5_GPIO_Port, col_6_GPIO_Port, col_7_GPIO_Port
};

static uint16_t col_pins[] = {
		col_0_Pin, col_1_Pin, col_2_Pin, col_3_Pin,
		col_4_Pin, col_5_Pin, col_6_Pin, col_7_Pin
};

static GPIO_TypeDef* debug_ports[] = {
		debug_1_GPIO_Port, debug_2_GPIO_Port,
		debug_3_GPIO_Port, debug_4_GPIO_Port
};

static uint16_t debug_pins[] = {
		debug_1_Pin, debug_2_Pin,
		debug_3_Pin, debug_4_Pin
};


/// Pre-defined mazes ///
//0: empty space; 1: wall; 2: start; 3: end
static uint8_t preset_mazes[4][8][8] = {
	{
		{2,0,0,0,0,0,1,0},
		{1,1,0,1,1,0,0,0},
		{0,0,0,0,0,0,1,0},
		{0,1,1,1,1,1,0,1},
		{0,0,0,0,1,0,0,0},
		{1,1,0,1,1,0,1,0},
		{0,0,0,1,0,0,1,0},
		{0,1,0,0,0,1,1,3}
	},
	{
		{2,0,0,0,0,0,0,0},
		{0,1,0,1,0,1,1,0},
		{0,0,1,0,1,0,0,0},
		{0,1,0,3,0,0,1,1},
		{0,0,1,1,1,1,0,0},
		{1,0,1,0,0,1,1,0},
		{0,0,1,0,0,0,1,0},
		{0,0,0,0,1,0,0,0}
	},
	{
		{2,0,0,1,1,0,0,0},
		{0,1,0,0,1,0,1,0},
		{0,0,1,0,1,0,1,0},
		{1,0,1,0,0,0,1,0},
		{0,0,1,0,1,1,1,0},
		{0,1,0,0,1,0,0,0},
		{0,1,0,1,1,0,1,1},
		{0,0,0,0,1,0,0,3}
	},
	{
		{2,1,3,1,0,0,0,0},
		{0,1,0,0,0,1,1,0},
		{0,1,1,1,1,0,0,0},
		{0,1,0,0,1,0,1,0},
		{0,1,0,1,0,0,1,0},
		{0,0,0,0,0,1,0,0},
		{1,1,1,1,1,1,0,1},
		{0,0,0,0,0,0,0,0}
	}
};


/// Display Numbers ///
// 1: LED ON; 2: LED OFF
static uint8_t preset_numbers[4][8][8] = {
		{
			{0,0,0,0,0,0,0,0},
			{0,0,0,1,0,0,0,0},
			{0,0,0,1,0,0,0,0},
			{0,0,0,1,0,0,0,0},
			{0,0,0,1,0,0,0,0},
			{0,0,0,1,0,0,0,0},
			{0,0,0,1,0,0,0,0},
			{0,0,0,0,0,0,0,0}
		},
		{
			{0,0,0,0,0,0,0,0},
			{0,0,0,1,1,0,0,0},
			{0,0,1,0,0,1,0,0},
			{0,0,0,0,0,1,0,0},
			{0,0,0,0,1,0,0,0},
			{0,0,0,1,0,0,0,0},
			{0,0,1,1,1,1,0,0},
			{0,0,0,0,0,0,0,0}
		},
		{
			{0,0,0,0,0,0,0,0},
			{0,0,1,1,0,0,0,0},
			{0,0,0,0,1,0,0,0},
			{0,0,1,1,0,0,0,0},
			{0,0,0,0,1,0,0,0},
			{0,0,0,0,1,0,0,0},
			{0,0,1,1,0,0,0,0},
			{0,0,0,0,0,0,0,0}
		},
		{
			{0,0,0,0,0,0,0,0},
			{0,0,0,0,0,0,1,0},
			{0,0,0,0,0,1,0,0},
			{0,0,0,0,1,0,0,0},
			{0,0,0,1,0,1,0,0},
			{0,0,1,1,1,1,1,0},
			{0,0,0,0,0,1,0,0},
			{0,0,0,0,0,0,0,0}
		}
};


/// Function Definitions ///


void light_up_LED(uint8_t x, uint8_t y) {
	HAL_GPIO_WritePin(row_ports[y], row_pins[y], GPIO_PIN_SET);
	HAL_GPIO_WritePin(col_ports[x], col_pins[x], GPIO_PIN_SET);
}

void reset_all_ports(uint8_t isBallSkipped) {

	// if isBallSkipped = 1: do not reset the ball
	// if isBallSKipped = 0: reset the ball light as well

	for (int i = 0; i < 8; ++i) {
		if (i != ball_y_pos || !isBallSkipped) {
			HAL_GPIO_WritePin(row_ports[i], row_pins[i], GPIO_PIN_RESET);
		}
	}
	for (int i = 0; i < 8; ++i) {
		if (i != ball_x_pos || !isBallSkipped) {
			HAL_GPIO_WritePin(col_ports[i], col_pins[i], GPIO_PIN_RESET);
		}
	}

}

// Reset all the debug leds to be off
void reset_debug_leds() {
	for (int debug_num = 0; debug_num < 4; ++debug_num) {
		HAL_GPIO_WritePin(debug_ports[debug_num], debug_pins[debug_num], GPIO_PIN_RESET);
	}
}

// Light up debug led; LED nums are 1,2,3,4
void light_debug_led(int led_num, uint8_t isReset) {
	// isReset = 1: Reset all debug leds first
	// isReset = 0: Leave all debug leds as is
	if (isReset) {
		reset_debug_leds();
	}
	// If the led_num is not allowed: return
	if (led_num < 1 || led_num > 4) {
		return;
	}
	else {
		// Turn on the desired debug led
		--led_num;
		HAL_GPIO_WritePin(debug_ports[led_num], debug_pins[led_num], GPIO_PIN_SET);
	}
}



// Copy the passed array to the current display
void copy_to_display(uint8_t buffer[8][8]) {
	for (int i = 0; i < 8; ++i) {
		for (int j = 0; j < 8; ++j) {
			display[i][j] = buffer[i][j];
		}
	}
}


// Load a preset maze to the display
void load_preset_maze(uint8_t maze_num) {
	if (!should_load_maze_flag) {
		return;
	}

	// Copy the maze to the display
	copy_to_display(preset_mazes[maze_num - 1]);

	// Set ball position for maze
	for (int row = 0; row < 8; ++row) {
		for (int col = 0; col < 8; ++col) {
			if (preset_mazes[maze_num - 1][row][col] == 2) {
				ball_x_pos = col;
				ball_y_pos = row;
			}
		}
	}
	should_load_maze_flag = 0;
}

// Load a preset number to the display
void load_preset_num_to_display(uint8_t preset) {
	if (!should_load_num_flag) {
		return;
	}
	else {
		copy_to_display(preset_numbers[--preset]);
		should_load_num_flag = 0;
	}
}

// Increment the column that should be displayed
void increment_column() {
	if (column_to_display < 7) {
		++column_to_display;
	}
	else {
		column_to_display = 0;
	}
}

// Display the current matrix through column sweeping
void display_matrix() {
	// Display MAZE_SELECTION
	if (matrixStatus == MAZE_SELECTION) {
		reset_all_ports(0);
		for (int row = 0; row < 8; ++row) {
			// Leds that make up number
			if (display[row][column_to_display] == 1) {
				light_up_LED(column_to_display, row);
			}
		}
		increment_column();
	}
	// Display maze game
	else if (matrixStatus == MAZE) {
		reset_all_ports(0);
		for (int row = 0; row < 8; ++row) {
			// Maze walls
			if (display[row][column_to_display] == 1) {
				light_up_LED(column_to_display, row);
			}
			// Maze ball
			else if ((row == ball_y_pos) && (column_to_display == ball_x_pos) && display_matrix_ball_flag) {
				light_up_LED(ball_x_pos, ball_y_pos);
			}
			// Maze end
			else if (display[row][column_to_display] == 3 && display_matrix_end_flag) {
				light_up_LED(column_to_display, row);
			}
		}
		increment_column();
	}

	// Display tennis game
	else if (matrixStatus == TENNIS) {

		/* Slider has been commented out
		//read_slider(); // Update bat y position
		*/

		reset_all_ports(0);
		for (int row = 0; row < 8; ++row) {
			// Display bat
			if (column_to_display == bat_x_pos && row == bat_y_pos) {
				light_up_LED(bat_x_pos, bat_y_pos);
				light_up_LED(bat_x_pos, bat_y_pos+1);
			}
			// Display ball
			if (column_to_display == ball_x_pos && row == ball_y_pos) {
				light_up_LED(ball_x_pos, ball_y_pos);
			}
		}
		increment_column();
	}
}

// Return 1 if the ball can move with the increment and 0.
// Check if end is reached and then set to corners display
uint8_t checkMazeBallMove(int x_increment, int y_increment) {

	// Ball can only move every 300ms
	if (HAL_GetTick() - ball_maze_move_time <= 300) {
		return 0;
	}

	int new_ball_x = ball_x_pos + x_increment;
	int new_ball_y = ball_y_pos + y_increment;

	if (display[new_ball_y][new_ball_x] == 1) { // move blocked by wall
		return 0;
	}
	else if (new_ball_x < 0 || new_ball_x > 7) { // out of left or right bounds
		return 0;
	}
	else if (new_ball_y < 0 || new_ball_y > 7) { // out of top or bottom bounds
		return 0;
	}
	else if (display[new_ball_y][new_ball_x] == 3) { // end is reached
		// Relies on the current loaded display for the end of the maze
		matrixStatus = CORNERS;
		reset_debug_leds();
		should_load_maze_flag = 1;
		return 1;
	}
	else {
		return 1;
	}

}

void move_ball_maze_imu() {
	uint8_t isMoved = 0;
	// Check horizontal move
	if (checkMazeBallMove(imu_direction_to_move[0], 0) && imu_direction_to_move[0] != 0) {
		ball_x_pos += imu_direction_to_move[0];
		isMoved = 1;
	}
	// Check vertical move
	if (checkMazeBallMove(0, imu_direction_to_move[1]) && imu_direction_to_move[1] != 0) {
		ball_y_pos += imu_direction_to_move[1];
		isMoved = 1;
	}
	// Update time if moved
	if (isMoved) {
		ball_maze_move_time = HAL_GetTick();
	}
}

uint8_t checkBatMove(int x_increment, int y_increment) {

	// Bat can only move every 100ms
	if (HAL_GetTick() - bat_move_time <= 100) {
		return 0;
	}

	int new_bat_x = bat_x_pos + x_increment;
	int new_bat_y = bat_y_pos + y_increment;

	if (new_bat_x < 0 || new_bat_x > 7) { // left/right wall
		return 0;
	}
	else if (new_bat_y < 0 || new_bat_y > 6) { // top/bottom wall
		return 0;
	}
	else {
		return 1;
	}

}

void move_bat_imu() {
	uint8_t isMoved = 0;
	// Check horizontal move
	if (checkBatMove(imu_direction_to_move[0], 0) && imu_direction_to_move[0] != 0) {
		bat_x_pos += imu_direction_to_move[0];
		isMoved = 1;
	}
	// Check vertical move
	if (checkBatMove(0, imu_direction_to_move[1]) && imu_direction_to_move[1] != 0) {
		bat_y_pos += imu_direction_to_move[1];
		isMoved = 1;
	}
	if (isMoved) { // update time
		bat_move_time = HAL_GetTick();
	}
}

// Move the tennis ball
void move_tennis_ball() {
	// Check current direction first
	// 0: East
	// 1: West
	// 2: South-East
	// 3: North-East
	// 4: North-East
	// 5: South-West
	int new_ball_x = ball_x_pos;
	int new_ball_y = ball_y_pos;

	/// Calculate the new ball position ///
	if (tennis_ball_direction == 0 || tennis_ball_direction == 3 || tennis_ball_direction == 5) { // Going left
		--new_ball_x;
		if (tennis_ball_direction == 3) { // Up
			--new_ball_y;
		}
		else if (tennis_ball_direction == 5) { // Down
			++new_ball_y;
		}
	}
	else { // Going right
		++new_ball_x;
		if (tennis_ball_direction == 4) { // Up
			--new_ball_y;
		}
		else if (tennis_ball_direction == 2) { // Down
			++new_ball_y;
		}
	}



	/// Check for bat collision ///
	uint8_t bat_upper_y = bat_y_pos;
	uint8_t bat_lower_y = bat_y_pos + 1;

	if (new_ball_x == bat_x_pos) { // Same x position: possible collision

		if (tennis_ball_direction == 3) { // diagonally up

			if (new_ball_y == bat_upper_y) { // leaves diagonally up
				tennis_ball_direction = 4;
				new_ball_x += 2;
				++tennis_ball_hit_counter;
			}
			else if (new_ball_y == bat_lower_y) { // hits corner; leaves diagonally down
				tennis_ball_direction = 2;
				new_ball_y += 2;
				new_ball_x += 2;
				++tennis_ball_hit_counter;
			}
			else if (ball_y_pos == bat_upper_y) { // leaves diagonally up
				tennis_ball_direction = 4;
				new_ball_x += 2;
				++tennis_ball_hit_counter;
			}


		}
		else if (tennis_ball_direction == 0) { // perpendicular

			if (new_ball_y == bat_upper_y) { // leaves diagonally up
				tennis_ball_direction = 4;
				new_ball_y -= 1;
				new_ball_x += 2;
				++tennis_ball_hit_counter;
			}
			else if (new_ball_y == bat_lower_y) { // leaves diagonally down
				tennis_ball_direction = 2;
				new_ball_y += 1;
				new_ball_x += 2;
				++tennis_ball_hit_counter;
			}

		}
		else if (tennis_ball_direction == 5) { // diagonally down

			if (new_ball_y == bat_lower_y) { // leaves diagonally down
				tennis_ball_direction = 2;
				new_ball_x += 2;
				++tennis_ball_hit_counter;
			}
			else if (new_ball_y == bat_upper_y) { // hits corner; leaves diagonally up
				tennis_ball_direction = 4;
				new_ball_y -= 2;
				new_ball_x += 2;
				++tennis_ball_hit_counter;
			}
			else if (ball_y_pos == bat_lower_y) { // leaves diagonally down
				tennis_ball_direction = 2;
				new_ball_x += 2;
				++tennis_ball_hit_counter;
			}

		}
	}




	/// Check for wall collision and calculate new direction ///
	/// and new position                                     ///
	if (new_ball_x > 7) { // Hit right wall
		if (tennis_ball_direction == 4) {
			tennis_ball_direction = 3;
		}
		else if (tennis_ball_direction == 1) {
			tennis_ball_direction = 0;
		}
		else if (tennis_ball_direction == 2) {
			tennis_ball_direction = 5;
		}
		new_ball_x = 6;
	}
	if (new_ball_y > 7) { // Hit bottom
		if (tennis_ball_direction == 5) {
			tennis_ball_direction = 3;
		}
		else if (tennis_ball_direction == 2) {
			tennis_ball_direction = 4;
		}
		new_ball_y = 6;
	}
	else if (new_ball_y < 0) { // Hit top
		if (tennis_ball_direction == 3) {
			tennis_ball_direction = 5;
		}
		else if (tennis_ball_direction == 4) {
			tennis_ball_direction = 2;
		}
		new_ball_y = 1;
	}


	// Check for lose (left wall collision)
	if (new_ball_x < 0) {
		matrixStatus = CORNERS;
		tennis_ball_hit_counter = 0;
		tennis_ball_velocity = 1;
		tennis_ball_direction = 0;
	}


	// Finally update ball position
	ball_x_pos = new_ball_x;
	ball_y_pos = new_ball_y;

	// Check the hit counter
	if (tennis_ball_hit_counter >= 3) {
		if (tennis_ball_velocity < 10) {
			++tennis_ball_velocity;
		}
		tennis_ball_hit_counter = 0;
	}

}

void update_imu_direction(int imu_x, int imu_y) {

	imu_direction_to_move[0] = 0;
	imu_direction_to_move[1] = 0;

	uint8_t IMU_dir_transmit = 'N';

	if (imu_x >= 8192) { // Left
		imu_direction_to_move[0] = -1;
		IMU_dir_transmit = 'L';
	}
	else if (imu_x <= -8192) { // Right
		imu_direction_to_move[0] = 1;
		IMU_dir_transmit = 'R';
	}

	if (imu_y >= 8192) { // Down
		imu_direction_to_move[1] = 1;
		IMU_dir_transmit = 'D';
	}
	else if (imu_y <= -8192) { // Up
		imu_direction_to_move[1] = -1;
		IMU_dir_transmit = 'U';
	}

	// Update MAZE UART
	maze[6] = IMU_dir_transmit;

	// Update TENNIS UART
	tennis[8] = IMU_dir_transmit;

	return;
}




















