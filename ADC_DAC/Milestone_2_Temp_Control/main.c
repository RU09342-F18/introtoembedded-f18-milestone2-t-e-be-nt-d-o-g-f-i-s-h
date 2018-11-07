#include <msp430.h> 
#include <math.h>

/**
 * Author: Jordan Alberico and Timothy Duong
 * Date Created: 11/7/18
 * Last Modified: 11/7/18
 * Milestone2_Temp_Control
 */

double thermistor;
double tVoltage;
double temp;
double ln;
double fahr;

extern void PWMSetup();
extern void TimerASetup();
extern void UartSetup();
extern void adcSetup();
extern void doMath();

int main(void)
{

    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    UartSetup();                              // Sets up Uart
    adcSetup();
    PWMSetup();                               // Sets up LEDs
    TimerASetup();                            // Sets up timers
    __bis_SR_register(GIE);       // Enter LPM0, interrupts enabled
   while (1)
   {
       if (fahr > 75)
            {
                TA0CCR1 = 0xFF;
            }
            else
            {
                TA0CCR1 = 0x00;
            }
       __delay_cycles(10000);
       ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
   }
}

void PWMSetup()
{
    P1SEL |= BIT2; // Sets port 1.2 to TimerA CCR1
    P1DIR |= BIT2; // Sets port 1.2 to output
}

void doMath()
{
    tVoltage = 0.000806 * ADC12MEM0;
    thermistor = 10000*((3.3/tVoltage)-1);
    ln = log(thermistor/10000);
    temp = 1/(.003354 + .000256985*ln);
    fahr = 1.8*(temp - 273) + 32;
}

void adcSetup()
{
    ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
    ADC12CTL1 = ADC12SHP;                     // Use sampling timer
    ADC12IE = 0x01;                           // Enable interrupt
    ADC12CTL0 |= ADC12ENC;
    P6SEL |= 0x01;                            // P6.0 ADC option select
    P1DIR |= 0x01;                            // P1.0 output
}

void TimerASetup()
{
    TA0CTL = TASSEL_2 + MC_1 + OUTMOD_7;
    // Configures the timer for SMClk, Timer in UP mode , and outmode of Reset/set

    TA0CCTL1 = OUTMOD_2; // Sets TACCR1 to toggle

    TA0CCR0 = 0xFF; // Sets TA0CCR0
    TA0CCR1 = 0x00; // Sets TA0CCR1
}

void UartSetup()
{
    P4SEL |= BIT4+BIT5;                       // P4.4,5 = USCI_A1 TXD/RXD
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;                     // SMCLK
    UCA1BR0 = 104;                            // 1MHz 9600 (see User's Guide)
    UCA1BR1 = 0;                              // 1MHz 9600
    UCA1MCTL |= UCBRS_1 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt
}



#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{

  switch(__even_in_range(UCA1IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
            TA0CCR1 = UCA1RXBUF;                //sets PWM
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6:     // Vector  6:  ADC12IFG0
      while (!(UCA1IFG&UCTXIFG));
      doMath();
      UCA1TXBUF =  (unsigned short int) fahr; // transmit value in ADC

    __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                           // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break;
  }
}
