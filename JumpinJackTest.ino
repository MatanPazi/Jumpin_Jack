/* Tested. Works well (July 16th).
   16Mhz CPU clk & clkI/O -> Prescaler of 4 -> 4Mhz
   1 OVF scenario will occur every 256 * 2 = 512 samples.
   1/4MHz * 512 = 128[us]   
   going over the 15 sample look-up table will take 128us * 15 = 1.92[ms] -> 520.833[Hz] / # of pole pairs = RPS
   Dead-time:   
   I'll take 250[ns]*8 = 2[us] dead time.


   16Mhz CPU clk & clkI/O -> Prescaler of 2 -> 8Mhz
   1 OVF scenario will occur every 256 * 2 = 512 samples.
   1/8MHz * 512 = 64[us]   
   going over the 15 sample look-up table will take 64us * 15 = 0.96[ms] -> 1041.666[Hz] / # of pole pairs = RPS
   Dead-time:   
   I'll take 125[ns]*8 = 1[us] dead time.
  
   // Setting the LED display
   https://lastminuteengineers.com/tm1637-arduino-tutorial/
   //Atmega328 pin numbers:
   http://www.learningaboutelectronics.com/Articles/Atmega328-pinout.php
   
   //
   //To-do *****************************************************************************
   Save last phase configuration to EEPROM. ********************************************
   *************************************************************************************
*/
#define _DISABLE_ARDUINO_TIMER0_INTERRUPT_HANDLER_  //These 2 lines were added to be able to compile. Also changed wiring.c file. Disables the previous overflow handles used for millis(), micros(), delay() etc.
#include <wiring.c>                                 //Reference: https://stackoverflow.com/questions/46573550/atmel-arduino-isrtimer0-ovf-vect-wont-compile-first-defined-in-vector/48779546
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

//
//millis(), delay() don't work as expected due to use of timers in PWM.
//Approx. time of loop ~
#define TEN_MS_OVF      160         // 160 OVF events (64[us]) results in 10[ms], which is the initial delay we need.
#define POSITIVE        1
#define NEGATIVE        -1

// Generated using: https://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
const uint8_t Sine[] = {0x7e,0xb4,0xe0,0xf8,0xf8,0xe0,0xb4,0x7e,0x47,0x1b,0x4,0x4,0x1b,0x47,0x7e};
const uint8_t Sine_Len = 15;              //Sine table length
//float Base_Freq = 520.8333;               // [Hz] Maximal frequency if the sine wave array index is incremented every OVF occurance. Prescaler = 4.
float Base_Freq = 1041.666;               //[Hz] Maximal frequency if the sine wave array index is incremented every OVF occurance. Prescaler = 2.

//const uint8_t Sine[] = {0x80,0x94,0xa8,0xbb,0xcc,0xdb,0xe8,0xf3,0xfa,0xfb,0xfb,0xfb,0xf7,0xee,0xe2,0xd4,0xc4,0xb1,0x9e,0x8a,0x75,0x61,0x4e,0x3b,0x2b,0x1d,0x11,0x8,0x4,0x4,0x4,0x5,0xc,0x17,0x24,0x33,0x44,0x57,0x6b,0x80};
//const uint16_t Sine_Len = 40;              //Sine table length
//float Base_Freq = 195.3125;               //[Hz] Maximal frequency if the sine wave array index is incremented every OVF occurance 

const uint8_t DT = 4;                     //Dead time to prevent short-circuit betweem high & low mosfets
volatile int8_t  Direction = 1;           //1: Positive, -1: Negative
uint32_t Init_PWM_Counter = 0;            //Used for charging the bootstrap capacitors, source below
volatile int16_t Sine_Index = 0;           //3 sine wave indices are used to allow for phase shifted sine waves.
volatile int16_t Sine_Index_120 = int16_t (Sine_Len / 3);
volatile int16_t Sine_Index_240 = int16_t((Sine_Len * 2) / 3);       //Sine_Len must be lower than 128, otherwise, change eq. 
volatile uint32_t OVF_Counter = 0;        //Increments every Timer0 overflow
volatile uint32_t Sine_Index_Counter = 0; //Counter, increments every time a sine index is incremented/decremented
volatile uint32_t Deg_In_Index = 0;                //Desired sine wave freq [Hz], updated below

float Desired_Freq = 0.0;                 //Desired motion in degrees in units of sine array indices. Updated below.
volatile uint32_t WaitCounter = 0;


// Variables to update:
float Num_Pole_Pairs = 7.0;   //# of pole pairs.
float RPS = 1.0;                    //Rotations per second
float Amp = 0.5;                    //Leave as 1.0, messes up DT
float Deg = 360.0;            //Degree of rotation
//

void setup()
{
  Desired_Freq = RPS * Num_Pole_Pairs;                          //Desired sine wave freq [Hz]
  Deg_In_Index = (unsigned long)(Deg * ((float(Sine_Len) * Num_Pole_Pairs) / 360.0));   //Desired motion in degrees in units of sine array indices  
  cli();                                      //Disable interrupts
  CLKPR = (1 << CLKPCE);                      //Enable change of the clock prescaler
//  CLKPR = (1 << CLKPS1);                      //Set system clock prescaler to 4. Beforehand DT had to be increased to a large value due to fall time of ~6 us of mosfet.
  CLKPR = (1 << CLKPS0);                      //Set system clock prescaler to 2.
  sei();
  Pwm_Config();
//  Serial.begin(2000000);                        //Set the baud rate to double that which is set in "Serial Monitor" due to the prescaler being 2 instead of 1.  
}
void loop()
{  

}

/* Wait_A_Bit(): Delays for a specified amount of executions
*/
void Wait_A_Bit(uint32_t Executions_To_Wait)
{
  volatile uint32_t Timer_Temp2 = 0;
  while (Timer_Temp2 < Executions_To_Wait)
  {
    Timer_Temp2++;
  }
}


void Pwm_Config()
{
   //***Check in scope. Need to make sure the pins are LOW prior to and after setting them to outputs so don't accidentally cause short in IPM.
  DDRD = (1 << DDD6) | (1 << DDD5) | (1 << DDD3); //Sets the OC0A, OC0B and OC2B pins to outputs
  DDRB = (1 << DDB3) | (1 << DDB2) | (1 << DDB1); //Sets the OC2A, OC1B and OC1A pins to outputs
  cli();                      //Disable interrupts
  //Synchronising all 3 timers 1st segment. Source: http://www.openmusiclabs.com/learning/digital/synchronizing-timers/index.html
  GTCCR = (1<<TSM)|(1<<PSRASY)|(1<<PSRSYNC); //Halt all timers
  //Timer 0
  TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << COM0B0) | (1 << WGM00); // Clear OC0A and set OC0B counting up. Waveform mode 1 (Table 14-8)
  TCCR0B = (1 << CS00);       //No prescaler
  TIMSK0 = (1 << TOIE0);      //Timer/Counter0 Overflow Interrupt Enable
  OCR0A = 0;     //Sign determined by set or clear at count-up. High-side IGBT OFF.
  OCR0B = 127;   //Sign determined by set or clear at count-up. Low-side IGBT 50% duty cycle to charge bootstrap cap.
  // Timer 1
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM10); // Clear OC1A and set OC1B counting up. Waveform mode 1 (Table 14-8)
  TCCR1B = (1 << CS10);       //No prescaler
  OCR1A = 0;     //Sign determined by set or clear at count-up. High-side IGBT OFF.
  OCR1B = 127;   //Sign determined by set or clear at count-up. Low-side IGBT 50% duty cycle to charge bootstrap cap.
  // Timer 2
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM20); // Clear OC0A and set OC0B counting up. Waveform mode 1 (Table 14-8)
  TCCR2B = (1 << CS20);       //No prescaler
  OCR2A = 0;     //Sign determined by set or clear at count-up. High-side IGBT OFF.
  OCR2B = 127;   //Sign determined by set or clear at count-up. Low-side IGBT 50% duty cycle to charge bootstrap cap.
  //Synchronising all 3 timers 2nd segment. Source: http://www.openmusiclabs.com/learning/digital/synchronizing-timers/index.html
  TCNT0 = 0;     //Set timer0 to 0
  TCNT1H = 0;    //Set timer1 high byte to 0
  TCNT1L = 0;    //Set timer1 low byte to 0
  TCNT2 = 0;     //Set timer2 to 0      
  GTCCR = 0;     //Release all timers
  sei();      
}
 

ISR (TIMER0_OVF_vect)
{   
  if (Init_PWM_Counter < TEN_MS_OVF) Init_PWM_Counter++;           //Charge bootstrap cap
  else
  {
    OVF_Counter++;   
    if (OVF_Counter >= ((unsigned long)(Base_Freq / Desired_Freq)))
    {
      if (Sine_Index_Counter <= Deg_In_Index)
      {
        if (Direction == POSITIVE)
        {
  //        Serial.println(Sine_Index);
          Sine_Index++;
          Sine_Index_120++;
          Sine_Index_240++;
          if (Sine_Index == Sine_Len) Sine_Index = 0;
          if (Sine_Index_120 == Sine_Len) Sine_Index_120 = 0;
          if (Sine_Index_240 == Sine_Len) Sine_Index_240 = 0;  
        }
        else if (Direction == NEGATIVE)
        {
  //        Serial.println(Sine_Index);
          Sine_Index--;
          Sine_Index_120--;
          Sine_Index_240--;
          if (Sine_Index < 0) Sine_Index = Sine_Len - 1;
          if (Sine_Index_120 < 0) Sine_Index_120 = Sine_Len - 1;
          if (Sine_Index_240 < 0) Sine_Index_240 = Sine_Len - 1;
        }
        //  
        Sine_Index_Counter++;

        if ((Amp * Sine[Sine_Index] - DT) < 0) OCR0A = 0;
        else OCR0A = uint8_t(Amp * Sine[Sine_Index] - DT);        

        if ((Amp * Sine[Sine_Index]) < 2*DT) OCR0B = 2*DT;
        else OCR0B = uint8_t(Amp * Sine[Sine_Index] + DT);
               
        if ((Amp * Sine[Sine_Index_120] - DT) < 0) OCR1A = 0;
        else OCR1A = uint8_t(Amp * Sine[Sine_Index_120] - DT);

        if ((Amp * Sine[Sine_Index_120]) < 2*DT) OCR1B = 2*DT;
        OCR1B = uint8_t(Amp * Sine[Sine_Index_120] + DT);
           
        if ((Amp * Sine[Sine_Index_240] - DT) < 0) OCR2A = 0;
        else OCR2A = uint8_t(Amp * Sine[Sine_Index_240] - DT);

        if ((Amp * Sine[Sine_Index_240]) < 2*DT) OCR2B = 2*DT;
        OCR2B = uint8_t(Amp * Sine[Sine_Index_240] + DT);
      }
      else
      {
        WaitCounter++;
        OCR0A = 0;
        OCR1A = 0;
        OCR2A = 0;
        OCR0B = 255;
        OCR1B = 255;
        OCR2B = 255;
        if (WaitCounter > 50)
        {          
          Direction = -Direction;
          Sine_Index_Counter = 0;    
          WaitCounter = 0;      
        }
      }
      OVF_Counter = 0;
    }
  }
//  Serial.print(OCR0A);
//  Serial.print(" ");
//  Serial.println(OCR0B);   
//  Serial.println(OVF_Counter);
}
