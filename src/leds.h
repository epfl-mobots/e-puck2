#ifndef LEDS_H
#define LEDS_H

#ifdef __cplusplus
extern "C" {
#endif

void e_set_led(unsigned int led_number, unsigned int value); // set led_number (0-7) to value (0=off 1=on higher=inverse) 
void e_led_clear(void);

void e_set_body_led(unsigned int value); // value (0=off 1=on higher=inverse) 
void e_set_front_led(unsigned int value); //value (0=off 1=on higher=inverse) 

#ifdef __cplusplus
}
#endif

#endif
