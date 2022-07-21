/* Tested. Works well (July 16th).

   16Mhz CPU clk & clkI/O -> Prescaler of 2 -> 8Mhz
   1 OVF scenario will occur every 256 * 2 = 512 samples.
   1/8MHz * 512 = 64[us]   
   going over the 15 sample look-up table will take 64us * 15 = 0.96[ms] -> 1041.666[Hz] / # of pole pairs = RPS
   Dead-time:   
   I'll take 125[ns]*8 = 1[us] dead time.
  
*/
#define _DISABLE_ARDUINO_TIMER0_INTERRUPT_HANDLER_  //These 2 lines were added to be able to compile. Also changed wiring.c file. Disables the previous overflow handles used for millis(), micros(), delay() etc.
#include <wiring.c>                                 //Reference: https://stackoverflow.com/questions/46573550/atmel-arduino-isrtimer0-ovf-vect-wont-compile-first-defined-in-vector/48779546
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

//
//millis(), delay() don't work as expected due to use of timers in PWM.
//Approx. time of loop ~
#define TEN_MS_OVF      160         // 160 OVF events (64[us]) results in 10[ms], which is the initial delay we need to charge bootstrap caps.
#define POSITIVE        1
#define NEGATIVE        -1

// Generated using: https://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
const uint8_t Sine[] = {0x7e,0xb4,0xe0,0xf8,0xf8,0xe0,0xb4,0x7e,0x47,0x1b,0x4,0x4,0x1b,0x47,0x7e};
const uint8_t Sine_Len = 15;              //Sine table length
float Base_Freq = 1041.666;               //[Hz] Maximal frequency if the sine wave array index is incremented every OVF occurance. Prescaler = 2.

const uint8_t DT = 4;                     //Dead time to prevent short-circuit betweem high & low mosfets
volatile int8_t  Direction = 1;           //1: Positive, -1: Negative
uint32_t Init_PWM_Counter = 0;            //Used for charging the bootstrap capacitors
volatile int16_t Sine_Index = 0;           //3 sine wave indices are used to allow for phase shifted sine waves.
volatile int16_t Sine_Index_120 = int16_t (Sine_Len / 3);
volatile int16_t Sine_Index_240 = int16_t((Sine_Len * 2) / 3);       //Sine_Len must be lower than 128, otherwise, change eq. 
volatile uint32_t OVF_Counter = 0;        //Increments every Timer0 overflow
volatile uint32_t Sine_Index_Counter = 0; //Counter, increments every time a sine index is incremented/decremented
volatile uint32_t Deg_In_Index = 0;                //Desired sine wave freq [Hz], updated below

float Desired_Freq = 0.0;                 //Desired motion in degrees in units of sine array indices. Updated below.
volatile uint32_t WaitCounter = 0;
volatile uint32_t WaitTime = 0;


// Variables to update:
float Num_Pole_Pairs = 7.0;   //# of pole pairs.
float RPS = 1.0;              //Rotations per second
float Amp = 0.5;              //Amplitude of sine waves
float Deg = 360.0;            //Degree of rotation
//

void setup()
{  
  Desired_Freq = RPS * Num_Pole_Pairs;                          //Desired sine wave freq [Hz]
  WaitTime = 16000 / ((uint32_t)(Base_Freq / Desired_Freq));    //Wait 1 sec. OVF every 64 [us] -> 16000 is 1 second & counter will increment only if OVF_Counter > (Base_Freq / Desired_Freq).
  Deg_In_Index = (unsigned long)(Deg * ((float(Sine_Len) * Num_Pole_Pairs) / 360.0));   //Desired motion in degrees in units of sine array indices  
  cli();                                      //Disable interrupts
  CLKPR = (1 << CLKPCE);                      //Enable change of the clock prescaler
  CLKPR = (1 << CLKPS0);                      //Set system clock prescaler to 2.
  sei();                                      //Enable interrupts
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
          Sine_Index++;
          Sine_Index_120++;
          Sine_Index_240++;
          if (Sine_Index == Sine_Len) Sine_Index = 0;
          if (Sine_Index_120 == Sine_Len) Sine_Index_120 = 0;
          if (Sine_Index_240 == Sine_Len) Sine_Index_240 = 0;  
        }
        else if (Direction == NEGATIVE)
        {
          Sine_Index--;
          Sine_Index_120--;
          Sine_Index_240--;
          if (Sine_Index < 0) Sine_Index = Sine_Len - 1;
          if (Sine_Index_120 < 0) Sine_Index_120 = Sine_Len - 1;
          if (Sine_Index_240 < 0) Sine_Index_240 = Sine_Len - 1;
        }
        //  
        Sine_Index_Counter++;

        if ((Amp * Sine[Sine_Index] - DT) < 0)
        {
           OCR0A = 0;
           OCR0B = 2*DT;
        }         
        else
        {
           OCR0A = uint8_t(Amp * Sine[Sine_Index] - DT);
           OCR0B = uint8_t(Amp * Sine[Sine_Index] + DT);
        }
     
        if ((Amp * Sine[Sine_Index_120] - DT) < 0)
        {
           OCR1A = 0;
           OCR1B = 2*DT;
        }         
        else
        {
           OCR1A = uint8_t(Amp * Sine[Sine_Index_120] - DT);
           OCR1B = uint8_t(Amp * Sine[Sine_Index_120] + DT);
        }
         
        if ((Amp * Sine[Sine_Index_240] - DT) < 0)
        {
           OCR2A = 0;
           OCR2B = 2*DT;
        }         
        else
        {
           OCR2A = uint8_t(Amp * Sine[Sine_Index_240] - DT);
           OCR2B = uint8_t(Amp * Sine[Sine_Index_240] + DT);
        }                 
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
        if (WaitCounter > WaitTime)
        {          
          Direction = -Direction;
          Sine_Index_Counter = 0;    
          WaitCounter = 0;      
        }
      }
      OVF_Counter = 0;
    }
  }
}
