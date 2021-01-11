/*
 * MSP430FR6989.c
 *  Biblioteka dla MSP430
 *  Created on: Mar 26, 2020
 *  Author: Remigiusz Golinski
 */


#include "MSP430FR6989.h"
#include <msp430.h>



// function to STOP watchdog
void WD_STOP(){
    WDTCTL = WDTPW | WDTHOLD;   // zatrzymaj watchdoga
}

/////////////////////////////////////////////////////////////////////////////
// function for init GPIO
void init_GPIO(){
PM5CTL0 &= ~LOCKLPM5;       // Wylaczenie blokady GPIO
}

// function to set BIT(bit) from port X as output X(bit)
void init_P1_out(unsigned int bit){
    P1DIR |= bit;            // ustaw wybrany bit jako wyjscie
    P1OUT |= bit;            // ustaw wybrane wyjscie
}


void init_P9_out(unsigned int bit){
    P9DIR |= bit;
    P9OUT |= bit;
}


//function to set MSP buttons S1 and S2 as outputs
void SET_MSP_Buttons(){
    P1REN |= BIT1;
    P1REN |= BIT2;
}

//function to set MSP button S1 as output with interrupt
void SET_MSP_Buttons_IE(unsigned int bit){

    P1REN |= bit;
    P1OUT |= bit;
    P1IE |= bit;                      //WÂ³Â¹czenie przerwania od P1.1
    P1IES |= bit;                     //WÂ³Â¹czenie przerwania na opadajÂ¹ce zbocze
    P1IFG &=~ bit;                    //Wyzerowanie flagi przerwania od P1.1
}

////////////////////////////////////////////////////////////////////////////////////



void PWMoutput(unsigned int bit){
    P1DIR |= bit;
    P1SEL0 |= bit;    // P1.0 jako wyjÅ›cie T1A0
}

void PWM(unsigned int X){
    TA0CCR0 = 100;    // do ilu zlicza
    TA0CCR1 = X;    // wypeÅ‚nienie
    TA0CCTL1 |= OUTMOD_7;   // tryb reset/set
    TA0CTL |= TASSEL_2 + MC_1;  // SMCLK, UP mode
}

void START_STOP_MSP_BUTTON_BLINKRED_05S(unsigned int nr){
    int last = 0;
        int onoff = 1;
        P1OUT &= ~BIT0;
        while(1)
        {
            if(last != (P1IN & nr))
            {
                last = (P1IN & nr);
                if(!(P1IN & nr))
                {
                    if(onoff == 1)
                    {
                        P1OUT &= ~BIT0;
                        onoff = 0;
                    }
                    else
                    {
                        P1OUT &= ~BIT0;
                        onoff = 1;
                    }
                }
            }
            if(onoff)
            {
                P1OUT ^= BIT0;
                __delay_cycles(500000);
            }
        }
}

void blink_green_05s(){
    //for(;;){
    P9OUT ^= BIT7;
    __delay_cycles(500000);
    //}
}

void timer_A1_2hz(){
    TA1CTL = TASSEL_2 + MC_1 + ID_3;        // SMCLK, UP mode, 1MHz/8 = 125kH
    TA1CCR0 = 62500;                         // 125kHz / 62500 = 2Hz
    TA1CCTL0 = CCIE;                         // przerwania od A0 wczone

    _BIS_SR(GIE);                // LPM0 z obsug przerwa

}

void wlacznik_przerwan(unsigned int nr){
    P1IE |= nr;                                //
    P1IES |= nr;                                // zbocze opadajce
    P1IFG &= ~nr;                    //Wyzerowanie flagi przerwania od P1.1
    _BIS_SR(GIE);
}

void UART_P3(){
    P3SEL0 |= BIT4 | BIT5;   // P3.4 i P3.5 jako TX i RX
    P3SEL1 &= ~(BIT4 | BIT5);  // P3.4 i P3.5 jako TX i RX
}

void ClockL0(){
    CSCTL0_H = 0xA5;    // odblokowanie ustawień zegara
    CSCTL1 = DCOFSEL_0;   // DCO = 1MHz
    CSCTL2 = SELS_3;    // SMCLK taktowane z DCOCLK
    CSCTL3 = DIVA_0 | DIVS_0 | DIVM_0;  // wyłączenie wszystkich dzielników
    CSCTL0_H = 0;    // blokada ustawień zegara
}

void UCA1(unsigned int A){
    UCA1CTLW0 = UCSWRST;   // software reset (blokada)
       UCA1CTLW0 |= UCSSEL_2;   // taktowanie z SMCLK
       UCA1BR0 = A;    // 1MHz/9600 = 104.1667
       UCA1MCTLW |= UCBRS3;   // tabela 30-4, str. 776, 0x11 = 3
       UCA1CTLW0 &= ~UCSWRST;   // odblokowanie
       UCA1IE |= UCRXIE;    // włączenie przerwań od RX

}


/*
 *
 BITS OPERATIONS

 &=   AND for setting all byte to 0/1
 |=    OR for set particular bits
 ^=   XOR Toggle bit
 ~    NOT
 &=~  NAND
 *
 *
 *
 *
 */






//EXAMPLES OF USING INTERRUPTS
/*

__interrupt void Timer_A (void)       //TA0 - Procedura obsugi przerwania od licznika
{
    TA0CTL = TASSEL_2 + MC_1 + ID_3;        // SMCLK, UP mode, 1MHz/8 = 125kH
          TA0CCR0 = 62500;                         // 125kHz / 62500 = 2Hz
          TA0CCTL0 = CCIE;                         // przerwania od A0 wczone

          _BIS_SR(LPM0_bits + GIE);

        P1IE |= nr;                                //
       P1IES |= nr;                                // zbocze opadajce
       P1IFG &= ~nr;                    //Wyzerowanie flagi przerwania od P1.1
       _BIS_SR(GIE);

    if(x == 0)
    {
        x = 1;

        P9OUT |= BIT7;
    }
    P9OUT ^= BIT7;

}
*/
/*

#pragma vector = PORT1_VECTOR

__interrupt void Port_1 (void)        //Procedura oi przerwania od P1.1
{
   TA0CCTL0 ^= CCIE;
   P1IFG &= ~BIT1;                    //Wyzerowanie fgi przerwania od P1.1
   x = 0;
   P1OUT &= ~nr;
   P9OUT &= ~BIT7;
}
*/


/*
 #pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
storedata[h++] = UCA0RXBUF;                       //String "storedata" is used to store Rx data
  if(storedata[h-1] == '\r')                                 //check if enter is pressed
    {

       storedata[h-1] = '\0';                                 //add null char to terminate string
       h=0; //reset address of data string
    }

 if(strcmp(storedata, prestring) == 0)           //Compare sting(here it is predefined and containing text)
{ P1OUT ^= 0x01;
                 // Toggle if string match. It can have any length
}
}

#pragma vector = PORT1_VECTOR
__interrupt void Port_1 (void)        //Procedura oi przerwania od P1.1
{


   if(x==0){
       x = 1;
   }
   else{
       x=0;
   }
   P1IFG &= ~BIT1;                    //Wyzerowanie fgi przerwania od P1.1

}

#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer_A (void)       //TA0 - Procedura obsugi przerwania od licznika
{


    if(x==1){
    P1OUT ^= BIT0;
    }

    else{
    P1OUT &=~BIT0;
    }


}
 *
 *
 */

/*
 *
 BITS OPERATIONS

 &=   AND for setting all byte to 0/1
 |=    OR for set particular bits
 ^=   XOR Toggle bit
 ~    NOT
 &=~  NAND
 *
 *
 *
 *
 */


