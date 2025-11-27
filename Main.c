#include <Arduino.h>
#include <msp430.h>
#include <stdio.h>
#include "i2c.h"
#include "ssd1306.h"

volatile char flagEncoder1 = 0, flagEncoder2 = 0;

unsigned int timeInterval1 = 0;
unsigned int timeInterval2 = 0;

float motorFreq1 = 0;
float motorFreq2 = 0;

const float REF_CLK        = 32768.0;   // ACLK
const float PULSES_PER_REV = 48.0;

// System clock til ~25 MHz
void init_SMCLK_25MHz(void) {
  WDTCTL = WDTPW | WDTHOLD;

  P5SEL |= BIT2 | BIT3 | BIT4 | BIT5;

  __bis_SR_register(SCG0);
  UCSCTL0 = 0x0000;
  UCSCTL1 = DCORSEL_7;
  UCSCTL2 = FLLD_0 + 610;
  __bic_SR_register(SCG0);

  do {
    UCSCTL7 &= ~(XT2OFFG | XT1LFOFFG | DCOFFG);
    SFRIFG1 &= ~OFIFG;
  } while (SFRIFG1 & OFIFG);

  UCSCTL3 = SELREF__REFOCLK;
  UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK | SELM__DCOCLK;
  UCSCTL5 = DIVS__1;
}

// PWM på P2.0 (TA1.1)
void init_pwm(void) {
  TA1CTL   = TASSEL_2 | MC_3;   // SMCLK, up/down mode
  TA1CCR0  = 1024;              // periode
  TA1CCR1  = 400;               // duty start
  TA1CCTL1 = OUTMOD_2;          // reset/set

  P2DIR |= BIT0;
  P2SEL |= BIT0;
}

// Capture på P1.2 (TA0.1) og P1.3 (TA0.2)
void init_capture(void) {
  TA0CTL   = TASSEL_1 | MC_2;   // ACLK, continuous

  TA0CCTL1 = CAP | CM_1 | CCIE | CCIS_0 | SCS; // encoder 1
  TA0CCTL2 = CAP | CM_1 | CCIE | CCIS_0 | SCS; // encoder 2

  P1SEL |= BIT2 | BIT3;
  P1DIR &= ~(BIT2 | BIT3);
}

static void show_rpm(uint8_t line, float freq) {
  float rps = freq / PULSES_PER_REV;
  float rpm = rps * 60.0f;
  char txt[20];
  sprintf(txt, "RPM%u: %.1f", (unsigned)line, rpm);
  ssd1306_printText(0, line, txt);
}

int main(void) {
  init_SMCLK_25MHz();
  init_pwm();
  init_capture();

  __delay_cycles(100000);

  i2c_init();
  ssd1306_init();
  reset_display();

  __enable_interrupt();

  char txt[20];

  while (1) {
    if (flagEncoder1) {
      flagEncoder1 = 0;
      show_rpm(1, motorFreq1);
    }

    if (flagEncoder2) {
      flagEncoder2 = 0;
      show_rpm(2, motorFreq2);
    }

    float duty = (float)TA1CCR1 / (float)TA1CCR0 * 100.0f;
    sprintf(txt, "Duty: %.1f%%", duty);
    ssd1306_printText(0, 0, txt);
  }
}

// Interrupt for begge encodere
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A0_ISR(void) {
  static unsigned int lastEdge1 = 0, lastEdge2 = 0;
  static char pulseCounter1 = 0, pulseCounter2 = 0;

  switch (TA0IV) {
    case 0x02:  // Encoder 1
      timeInterval1 = (TA0CCR1 >= lastEdge1)
                      ? (TA0CCR1 - lastEdge1)
                      : (65535 - lastEdge1 + TA0CCR1);
      lastEdge1 = TA0CCR1;
      pulseCounter1++;
      if (pulseCounter1 >= 2) {
        motorFreq1 = REF_CLK / (float)timeInterval1;
        pulseCounter1 = 0;
        flagEncoder1 = 1;
      }
      break;

    case 0x04:  // Encoder 2
      timeInterval2 = (TA0CCR2 >= lastEdge2)
                      ? (TA0CCR2 - lastEdge2)
                      : (65535 - lastEdge2 + TA0CCR2);
      lastEdge2 = TA0CCR2;
      pulseCounter2++;
      if (pulseCounter2 >= 2) {
        motorFreq2 = REF_CLK / (float)timeInterval2;
        pulseCounter2 = 0;
        flagEncoder2 = 1;
      }
      break;
  }
}
