/*
 * MSP430FR6989.h
 *  Biblioteka dla MSP430
 *  Created on: Mar 26, 2020
 *  Author: Remigiusz Golinski
 */

#ifndef MSP430FR6989_H_
#define MSP430FR6989_H_

//Funkcja obslugi licznika Watchdog
void WD_STOP(void);

//inicjalizacja GPIO
void init_GPIO(void);
void init_P1_out(unsigned int bit);
void init_P9_out(unsigned int bit);

void SET_MSP_Buttons_IE(unsigned int bit);

void PWMoutput(unsigned int bit);

void SET_MSP_Buttons(void);

void START_STOP_MSP_BUTTON_BLINKRED_05S(unsigned int nr);

void timer_A1_2hz(void);

void wlacznik_przerwan(unsigned int nr);

void blink_green_05s(void);

void PWM(unsigned int X);

void UART_P3(void);

void ClockL0(void);

void UCA1(unsigned int A);



#endif /* MSP_LIB_REM_H_ */

