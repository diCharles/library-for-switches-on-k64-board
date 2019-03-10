
 
/**
 * @file    sw2_and_sw3.c
 * @brief   small example of implementation of library for  sw 2 and sw3
 */

#include "MK64F12.h"
#include "rgb.h"
#include "switches_k64.h"

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
int main(void) {


	init_rgb();
	/* interrupts not used for sw3, his state must be read manually*/
	init_sw2(NO_PRIORITY_TH,NO_PRIORITY,NO_INT_EDGE_DEFINED);
	/*    priority threshold=10,priority number 4, interrupt launched as button is pressed */
	init_sw3(PRIORITY_10,PRIORITY_4,INTR_FALLING_EDGE);

	while(1)
	{
		sw2_pressed();//reads sw2 state
		/*sw3 state is processed on the ISR for the port A*/
		if(sw2_one_shot())
		{
			rgb_color(BLUE,TOOGLE);
		}
		if(TRUE == GPIO_get_irq_status(GPIO_A))
		{
			rgb_color(RED,TOOGLE);
			GPIO_clear_irq_status(GPIO_A);
		}
	}
    return 0 ;
}
