#include "ir.h"

int read_ir(){
	gpio_reset_pin(GPIO_NUM_5);
	gpio_set_direction(GPIO_NUM_5,GPIO_MODE_INPUT);	

	int out = gpio_get_level(GPIO_NUM_5);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	return out;

}

