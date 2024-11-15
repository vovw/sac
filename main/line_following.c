#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"
#include "ir.h"

// Add these definitions at the top
#define ALL_BLACK_THRESHOLD 1000  // Threshold for detecting all black
#define ALL_WHITE_THRESHOLD 100  // Threshold for detecting all white
#define INTERSECTION_SENSOR_COUNT 3  // Minimum number of sensors needed to detect intersection

// Add these function declarations
bool is_intersection();
bool can_turn_left();
bool can_go_straight();
bool is_all_black();
bool should_turn_180();


#define MODE NORMAL_MODE
#define BLACK_MARGIN 4095
#define WHITE_MARGIN 0
#define bound_LSA_LOW 0
#define bound_LSA_HIGH 1000
#define BLACK_BOUNDARY  930    // Boundary value to distinguish between black and white readings
#define ROTATION_SPEED 65
int all_black_counter = 0;

/*
 * weights given to respective line sensor
 */
const int weights[5] = {-5, -3, 1, 3, 5};

/*
 * Motor value boundts
 */
int optimum_duty_cycle = 60;
int lower_duty_cycle = 45;
extern int higher_duty_cycle;

int higher_duty_cycle = 70;


float left_duty_cycle = 35, right_duty_cycle = 35;


void disable_motors(){
    motor_handle_t motor_a_0, motor_a_1;

    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_0, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_1, MOTOR_A_1));

    set_motor_speed(motor_a_0, MOTOR_FORWARD, 60);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, 60);

    vTaskDelay(1000);


    set_motor_speed(motor_a_0, MOTOR_FORWARD, 0);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, 0);

}

/*
 * Line Following PID Variables
 */
float error=0, prev_error=0, difference, cumulative_error, correction;

/*
 * Union containing line sensor readings
 */
line_sensor_array line_sensor_readings;

bool is_white_detected() {
    for(int i = 0; i < 5; i++) {
        if(line_sensor_readings.adc_reading[i] < BLACK_BOUNDARY) {
            return true;
        }
    }
    return false;
}



void calculate_correction()
{
    error = error*10;  // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
    difference = error - prev_error;
    cumulative_error += error;

    cumulative_error = bound(cumulative_error, -40, 40);

    correction = read_pid_const().kp*error + read_pid_const().ki*cumulative_error + read_pid_const().kd*difference;
    prev_error = error;
}

void calculate_error()
{
    int all_black_flag = 1; // assuming initially all black condition
    float weighted_sum = 0, sum = 0;
    float pos = 0; int k = 0;

    for(int i = 0; i < 5; i++)
    {
	if(line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
	{
	    all_black_flag = 0;
	}
	if(line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
	{
	    k = 1;
	}
	if(line_sensor_readings.adc_reading[i] < BLACK_BOUNDARY)
	{
	    k = 0;
	}
	weighted_sum += (float)(weights[i]) * k;
	sum = sum + k;
    }

    if(sum != 0) // sum can never be 0 but just for safety purposes
    {
	pos = (weighted_sum - 1) / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
    }

    if(all_black_flag == 1)
    {
	all_black_counter++;
	if(all_black_counter>=8){
		error = 999;
	}
	else{
	if(prev_error > 0)
        {
            error = 2.5;
        }
        else
        {
            error = -2.5;
        }
	}
         // Special error value to indicate all-black condition
    }
    else
    {
	all_black_counter = 0;
        error = pos;
    }
}



bool is_intersection() {
    int black_count = 0;
    for(int i = 0; i < 5; i++) {
        if(line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY) {
            black_count++;
        }
    }
    return black_count >= INTERSECTION_SENSOR_COUNT;
}

bool can_turn_left() {
    // Check if leftmost sensor detects black
    return line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY;
}

bool can_go_straight() {
    // Check if center sensor detects black
    return line_sensor_readings.adc_reading[2] > BLACK_BOUNDARY;
}

bool is_all_black() {
    for(int i = 0; i < 5; i++) {
        if(line_sensor_readings.adc_reading[i] < BLACK_BOUNDARY) {
            return false;
        }
    }
    return true;
}

bool should_turn_180() {
    // Check if all sensors detect white (dead end)
    for(int i = 0; i < 5; i++) {
        if(line_sensor_readings.adc_reading[i] > ALL_WHITE_THRESHOLD) {
            return false;
        }
    }
    return true;
}

// Modify the line_follow_task function
void line_follow_task(void* arg)
{
    // ... (keep existing initialization code)
    motor_handle_t motor_a_0, motor_a_1;
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_0, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_1, MOTOR_A_1));
    adc_handle_t line_sensor;
    ESP_ERROR_CHECK(enable_line_sensor(&line_sensor));
    ESP_ERROR_CHECK(enable_bar_graph());
#ifdef CONFIG_ENABLE_OLED
    // Initialising the OLED
    ESP_ERROR_CHECK(init_oled());
    vTaskDelay(100);

    // Clearing the screen
    lv_obj_clean(lv_scr_act());

#endif


    while(true)
    {
        line_sensor_readings = read_line_sensor(line_sensor);
        for(int i = 0; i < 5; i++)
        {
            line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
        }

        if(should_turn_180()) {
            // Perform 180-degree turn
            //
            calculate_error();
            calculate_correction();

            set_motor_speed(motor_a_0, MOTOR_FORWARD, ROTATION_SPEED);
            set_motor_speed(motor_a_1, MOTOR_BACKWARD, ROTATION_SPEED);

            // Continue rotating until white line is detected
            while (!is_white_detected()) {
                line_sensor_readings = read_line_sensor(line_sensor);
                for(int i = 0; i < 5; i++) {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                    line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
        else if(is_intersection()&& read_ir()==0) {
            calculate_error();
            calculate_correction();

            if(can_turn_left()) {
                // Turn left
                set_motor_speed(motor_a_0, MOTOR_BACKWARD, ROTATION_SPEED);
                set_motor_speed(motor_a_1, MOTOR_FORWARD, ROTATION_SPEED);
            }
            else if(can_go_straight()) {
                // Go straight
                set_motor_speed(motor_a_0, MOTOR_FORWARD, optimum_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_FORWARD, optimum_duty_cycle);
            }
            else {
                // Turn right
                set_motor_speed(motor_a_0, MOTOR_FORWARD, ROTATION_SPEED);
                set_motor_speed(motor_a_1, MOTOR_BACKWARD, ROTATION_SPEED);
            }
        }
	else if(is_intersection()&& read_ir()!=0){
		calculate_error();
            	calculate_correction();

            	set_motor_speed(motor_a_0, MOTOR_FORWARD, ROTATION_SPEED);
            	set_motor_speed(motor_a_1, MOTOR_BACKWARD, ROTATION_SPEED);

            // Continue rotating until white line is detected
            while (!is_white_detected()) {
                line_sensor_readings = read_line_sensor(line_sensor);
                for(int i = 0; i < 5; i++) {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                    line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }



	}
        else {
            // Normal line following behavior
            calculate_error();
            calculate_correction();

            if (correction > 0) {
                left_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);
                right_duty_cycle = bound(correction, lower_duty_cycle, higher_duty_cycle);
                set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_BACKWARD, right_duty_cycle);
            } else if (correction < 0) {
                left_duty_cycle = bound(-correction, lower_duty_cycle, higher_duty_cycle);
                right_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
                set_motor_speed(motor_a_0, MOTOR_BACKWARD, left_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
            } else {
                left_duty_cycle = right_duty_cycle = optimum_duty_cycle;
                set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}
