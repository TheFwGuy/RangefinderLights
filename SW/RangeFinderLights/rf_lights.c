/**
 *  @file rf_lights.c
 *  @brief Handlimg the lights for the RangeFinder (rf) using PWM with a 
 *  MSP430F2012 Texas Instrument microcontroller
 *  @author Stefano B.
 *  @version 01 beta
 *  @date June 2008
 *  @details This code is designed to handle three LEDs with some light effects.
 *
 *  Version features
 *  A tilt switch activate the lights, with  some sequential patterns
 *
 *  The timer will be set in UP mode (i.e. counting up to the value in CCR0).
 *  The timer will generate an interrupt every .01 ms
 *  Internal management (SW counter) will generate the PWM outputs.
 *
 *  Pinout  PCB Pin   Mode  Description
 *  P1.0      P2      Out   Visor LED
 *  P1.1      P3      Out   Finder LED 1                                   
 *  P1.2      P4      Out   Finder LED 2
 *  P1.3      P5      Out   Unused
 *  P1.4      P6      SMCLK Debug - report the SMCLK frequency
 *  P1.5      P7      Out   Unsused 
 *  P1.6      P8      Out   Unused 
 *  P1.7      P9      Out   Unused
 *  P2.6      P13     In    Tilt switch 
 *  P2.7      P12     In    Unused
 *
 *  Built with IAR Embedded Workbench Version: 3.40A
 */

#include <string.h>
#include "msp430x20x2.h"

/*
 *  Functions prototype
 */
__interrupt void Timer_A (void);    /* Timer A0 interrupt service routine */
void read_pattern(unsigned char);
void LED_loading(unsigned char, unsigned short, unsigned short, unsigned short);
void LED_handling(unsigned char);
void Init(void);                    /* Init LED */
unsigned char testButton(unsigned char);

/*
 *  Global defines
 */

/* 
 *  The idea is to generate a PWM at 50 Hz with small steps to adjust the
 *  duty cycle.
 *  The Servomotor accept pulses between 380 uSec and 2.320 mSec.
 *  The frequency of the PWM (usually 50 Hz) set the actuation speed and the feedback
 *  response, so is better to have it set to 50 Hz.
 */
#define TMRVALUE        160      /* Timer will generate a interrupt every .01 ms */
#define PWMINITIALVALUE 0        /* Initial value duty cicle */
#define PWM1_MAXSTEP    2000     /* Maximum step for the PWM 1 */
#define PWM2_MAXSTEP    2000     /* Maximum step for the PWM 1 */
#define PWM3_MAXSTEP    2000     /* Maximum step for the PWM 1 */

#define DTOFF 1
#define DTQT   500
#define DTHALF 1000
#define DT3QT  1500
#define DTFULL 2000
/*
 *  Each LED has a pattern
 *  The pattern is divided in "triplets", indicating the duty cycle value to reach (0 ... 2000), 
 *  the changing speed (in 0.01 ms resolution) and the resting time (in .01 ms resolution.
 *  The pattern is ended with FFFF that indicate to repeat it again.
 */
#define VISOR_PATTERN   {DTQT,1,0,DT3QT,1,0, 0xFFFF}
#define LED1_PATTERN    {DTOFF,0,25000, DTFULL,0,25000, DTOFF,0,25000, DTFULL,0,25000, 50,20,25000, DTFULL,20,0, 0xFFFF}
#define LED2_PATTERN    {DTOFF,0,25000, DTOFF,0,25000, DTOFF,0,25000, DTOFF,0,25000, DTFULL,20,0, 50,20,25000, 0xFFFF}

#define PRESCALER       100      /* Value to obtain a 1 ms timing */

#define FALSE         0
#define TRUE          1
/*
 *  State machine states
 */
#define POSIT         0
#define MOVINGUP      1
#define MOVINGDOWN    2
#define WAITINGUP     3
#define WAITINGDOWN   4
#define ACTIVATE      5
#define RESTING       6

/* I/O defines */

#define S1_BUTTON 0
#define S2_BUTTON 1

#define VISOR_ON  P1OUT |= BIT0
#define VISOR_OFF P1OUT &= ~BIT0
#define VISOR_TOGGLE P1OUT ^= BIT0

#define LED1_ON  P1OUT |= BIT1
#define LED1_OFF P1OUT &= ~BIT1
#define LED1_TOGGLE P1OUT ^= BIT1

#define LED2_ON  P1OUT |= BIT2
#define LED2_OFF P1OUT &= ~BIT2
#define LED2_TOGGLE P1OUT ^= BIT2


/*
 *  Debug pins
 */
#define TEST0_ON  P1OUT |= BIT3
#define TEST0_OFF P1OUT &= ~BIT3
#define TEST0_TOGGLE P1OUT ^= BIT3

#define TEST_ON  P1OUT |= BIT5
#define TEST_OFF P1OUT &= ~BIT5
#define TEST_TOGGLE P1OUT ^= BIT5


/*
 *  Global variables
 */
unsigned short Pwm_reach[3];    /* PWM position to reach */
unsigned short Pwm_dc[3];       /* PWM output ducty cycle */
unsigned short Pwm_cn[3];       /* PWM counter */
unsigned char  Pwm_State[3];    /* PWM state machine */
unsigned short Pwm_delay[3];    /* Used for PWM delay */
unsigned short Pwm_speed[3];
unsigned short Pwm_rest[3];

unsigned short RfPrescaler;     /* Prescaler for long delays */
unsigned short RfLongDelay;     /* Counter for long delay - 1000 = 1 Sec. (1 ms - 65 sec) */
unsigned short RfShortDelay;    /* Counter for short delay - 1000 = 0.01 Sec */

unsigned char  Pattern[3];

const unsigned short Pattern1[] = VISOR_PATTERN;
const unsigned short Pattern2[] = LED1_PATTERN;
const unsigned short Pattern3[] = LED2_PATTERN;

/*
 *  Main entry file
 */
void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;             // Stop watchdog timer

  Init(); 

  RfLongDelay = 5000;       /* 5 Seconds delay */
  for(;;)
  {
     /*
      *  The tilt switch starts/stops the lights                                             
      *
      */
//     if(testButton(S2_BUTTON))
//     {
        if(!RfLongDelay)
        {
           RfLongDelay = 5000;    /* Reload 5 seconds delay */
           /*
            *  Every 5 cycles, force a restart for the two LEDs that need to be
            *  in sync
            */
           Pwm_cn[1]    = 0;                /* Reset virtual PWM 1 counter */
           Pwm_dc[1]    = PWMINITIALVALUE;  /* Set default PWM 1 value */
           Pwm_reach[1] = PWMINITIALVALUE;
           Pwm_State[1] = POSIT;
           Pattern[1]   = 0;

           Pwm_cn[2]    = 0;                /* Reset virtual PWM 1 counter */
           Pwm_dc[2]    = PWMINITIALVALUE;  /* Set default PWM 1 value */
           Pwm_reach[2] = PWMINITIALVALUE;
           Pwm_State[2] = POSIT;
           Pattern[2]   = 0;
        }
     
        LED_handling(0);
        LED_handling(1);
        LED_handling(2);
     }
//   }
}

/*
 * read_pattern
 * @brief The function read the pattern and assign the value
 *
 * @param led_id  Identifier for the LED to manage - 0 Visor - 1 LED 1 - 2 LED 2
 * @return None
 */
void read_pattern(unsigned char led_id)
{
   switch(led_id)
   {
      case 0:     /* Visor LED */
         LED_loading(led_id, Pattern1[Pattern[led_id]], Pattern1[Pattern[led_id]+1], Pattern1[Pattern[led_id]+2]);
         Pattern[led_id] += 3;     /* Point next sequence */
         if(Pattern1[Pattern[led_id]] == 0xFFFF)
            Pattern[led_id] = 0;
         break;
         
      case 1:     /* LED 1 */
         LED_loading(led_id, Pattern2[Pattern[led_id]], Pattern2[Pattern[led_id]+1], Pattern2[Pattern[led_id]+2]);
         Pattern[led_id] += 3;     /* Point next sequence */
         if(Pattern2[Pattern[led_id]] == 0xFFFF)
            Pattern[led_id] = 0;
         break;
            
      case 2:     /* LED 2 */
         LED_loading(led_id, Pattern3[Pattern[led_id]], Pattern3[Pattern[led_id]+1], Pattern3[Pattern[led_id]+2]);
         Pattern[led_id] += 3;     /* Point next sequence */
         if(Pattern3[Pattern[led_id]] == 0xFFFF)
            Pattern[led_id] = 0;
         break;
   }   

   return;
}


/*
 * LED_loading
 * @brief The function load the PWM assignment for a LED
 *
 * @param led_id  Identifier for the LED to manage - 0 Visor - 1 LED 1 - 2 LED 2
 * @param pwm_reach  PWM value to be reached
 * @return None
 */
void LED_loading(unsigned char led_id, unsigned short pwm_reach, unsigned short speed, unsigned short rest)
{
   /*
    *  Assign the reaching goal and the direction (increment
    *  or decrement)
    */
   Pwm_reach[led_id] = pwm_reach;
   
   if(speed)
   {
      if(Pwm_reach[led_id] == Pwm_dc[led_id])
         Pwm_State[led_id] = RESTING;
      else if(Pwm_reach[led_id] > Pwm_dc[led_id])
         Pwm_State[led_id] = MOVINGUP;
      else
         Pwm_State[led_id] = MOVINGDOWN;
   }
   else   
      Pwm_State[led_id] = ACTIVATE;

   Pwm_speed[led_id] = speed;
   Pwm_delay[led_id] = speed;    /* First load */
   Pwm_rest[led_id]  = rest;
   return;
}

/*
 * LED_handling
 * @brief The function handle the PWM assignment for a LED
 *
 * @param led_id  Identifier for the LED to manage - 0 Visor - 1 LED 1 - 2 LED 2
 * @return None
 */
void LED_handling(unsigned char led_id)
{
   switch(Pwm_State[led_id])
   {
      default:
         Pwm_State[led_id] = POSIT;
         break;
        
      case POSIT:
         read_pattern(led_id);
         break;
         
      case MOVINGUP:      
         if(Pwm_dc[led_id] == Pwm_reach[led_id] && Pwm_delay[led_id] == 0)
         {
            Pwm_State[led_id] = RESTING;
            Pwm_delay[led_id] = Pwm_rest[led_id];
         }  
         else
         {  
            if(Pwm_speed[led_id])
            {
               Pwm_State[led_id] = WAITINGUP;
               Pwm_delay[led_id] = Pwm_speed[led_id];
            }   
            Pwm_dc[led_id]++;
         }  
         break;

      case MOVINGDOWN:              
         if(Pwm_dc[led_id] == Pwm_reach[led_id] && Pwm_delay[led_id] == 0)
          {
            Pwm_State[led_id] = RESTING;
            Pwm_delay[led_id] = Pwm_rest[led_id];
         }  
         else
         {  
            if(Pwm_speed[led_id])
            {
               Pwm_State[led_id] = WAITINGDOWN;
               Pwm_delay[led_id] = Pwm_speed[led_id];
            }
            Pwm_dc[led_id]--;
         }  
         break;             

      case WAITINGUP:              
         if(Pwm_delay[led_id] == 0)
            Pwm_State[led_id] = MOVINGUP;
         break;             

      case WAITINGDOWN:              
         if(Pwm_delay[led_id] == 0)
            Pwm_State[led_id] = MOVINGDOWN;
         break;             

      case ACTIVATE:              
         Pwm_State[led_id] = RESTING;
         Pwm_delay[led_id] = Pwm_rest[led_id];
         Pwm_dc[led_id] = Pwm_reach[led_id];
         break;             
         
      case RESTING:              
         if(Pwm_delay[led_id] == 0)
            Pwm_State[led_id] = POSIT;
         break;             
   }
   return;
}


/*
 *  Init
 * @brief Initclock, timer and initialize variables
 *
 * @param none
 * @return None
 */
void Init(void)
{
  unsigned char index;
  
  WDTCTL = WDTPW + WDTHOLD;     /* Stop watchdog timer */

  /*
   *  Set DCO
   */
  DCOCTL  = CALDCO_16MHZ;
  BCSCTL1 = CALBC1_16MHZ;       /* Set up 16 Mhz using internal calibration value */

  /*
   *  Set I/O
   *  P1.0 -> I/O output - original LED on the board
   *  P1.2 -> Timer_A output - PWM out
   *  P2.6 <- input  - button 1 - Increase
   *  P2.7 <- input  - button 2 - Decrease
   */
  P1OUT = 0;                  /* Force out low */

  P1SEL = 0;                  /* Set Port 1 for normal I/O */
  P1SEL |= BIT4;              /* Set P1.4 on SMCLK for test */
  
  P2SEL &= ~BIT6;
  P2SEL &= ~BIT7;

  P1DIR |= 0xFF;              /* Set all in OUTPUT */
  P2DIR  = 0x00;              /* Leave P2.6 and P2.7 in input direction */

//  P1REN |= BIT6;              /* Enable Pull-up/Down resistor on P1.6 - P1OUT is 0 so is a pull-down */
//  P1IES &= ~BIT6;             /* Set P1.6 interrupt generation on the low-to-high transition */  
//  P1IE |= BIT6;               /* Enable interrupt on P1.6 */
  /*
   *  Set variables
   */
  for(index=0; index<2; index++)
  {
     Pwm_cn[index]    = 0;                /* Reset virtual PWM 1 counter */
     Pwm_dc[index]    = PWMINITIALVALUE;  /* Set default PWM 1 value */
     Pwm_reach[index] = PWMINITIALVALUE;
     Pwm_State[index] = POSIT;
     Pattern[index] = 0;
  }

  /*
   *  Set Timer
   */
  TACTL = TASSEL_2 + MC_1;    /* Uses SMCLK, count in up mode */
  TACCTL0 = CCIE;             /* Use TACCR0 to generate interrupt */
  TACCR0 = TMRVALUE;          /* Approx .01 ms */

  /*  NORMAL MODE */
  TACCTL0 &= ~0x0080;         /* Disable Out0 */

  _BIS_SR(GIE);               /* Enable interrupt */
}

/**
 * testButton
 * @brief Checking the pushbutton status
 *
 * The function is reading a pushbutton and return it's status.
 * Just to avoid multiple readings, the function is returning TRUE
 * (i.e. pushbutton pressed) only if detect a transition from true to
 *  false, with at least some delay.
 *  Primitive but for test purpose is enough.
 *
 * @param button button to be checked. 0 -> S1  1 -> S2
 * @return True if button is pressed, False if not pressed
 */
unsigned char testButton(unsigned char button)
{
   unsigned char retvalue = 0;
   unsigned char smallDelay = 0;
   
   retvalue = P2IN & (0x80 >> button);
   
   if(retvalue)
   {
//      for (smallDelay=200; smallDelay>0; smallDelay--);
           
      retvalue = P2IN & (0x80 >> button);
      if(retvalue)
      {
         return(TRUE);    
      }
      else
        return(FALSE);
   }
   else
      return (FALSE);
}

/**
 * Timer_A
 * @brief Timer A0 interrupt service routine
 *
 * This function handle the Timer A interrupt, in order to detect the RF
 * signal and drive the PWM output
 *
 * The functions are based on a state machine in order to optimize the 
 * operations, so to don't have too long operations under interrupt.<br>
 * The timer is set to generate an interrupt every 0.01 ms.
 *
 * @param none 
 * @return None
 */
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A( void )
{
  TEST0_TOGGLE;

  /*
   *  Delay management
   *  This counter is used in the states WAITUP and WAITDOWN in order to
   *  create a delay in the PWM generation
   */
  if(Pwm_delay[0])
    Pwm_delay[0]--;

  if(Pwm_delay[1])
    Pwm_delay[1]--;

  if(Pwm_delay[2])
    Pwm_delay[2]--;

  /*
   *  Delay management for detection confirmation
   */
  if(RfShortDelay)
    RfShortDelay--;

  /*
   *  Prescaler management for long delays
   */
  if(RfPrescaler)
    RfPrescaler--;
  else
  {
    RfPrescaler = PRESCALER;
    if(RfLongDelay)
      RfLongDelay--;
  }  
  
  /*
   *  PWM 1 management
   *  This part of the code is handling the PWM 1 generation for the visor LED
   */
  if(Pwm_cn[0] < PWM1_MAXSTEP)
  {
    Pwm_cn[0]++;
    if(Pwm_cn[0] <= Pwm_dc[0])
      VISOR_ON;
    else
      VISOR_OFF;
  }  
  else
  {
   /*
    *  Expired counter
    *  Reload counters - force starting output
    */   
    Pwm_cn[0] = 0;
    VISOR_OFF;   
  }  

  /*
   *  PWM 2 management
   *  This part of the code is handling the PWM 2 generation for the LED 1
   */
  if(Pwm_cn[1] < PWM2_MAXSTEP)
  {
    Pwm_cn[1]++;
    if(Pwm_cn[1] <= Pwm_dc[1])
      LED1_ON;
    else
      LED1_OFF;
  }  
  else
  {
   /*
    *  Expired counter
    *  Reload counters - force starting output
    */   
    Pwm_cn[1] = 0;
    LED1_OFF;   
  }  

  /*
   *  PWM 3 management
   *  This part of the code is handling the PWM 3 generation for the LED 2
   */
  if(Pwm_cn[2] < PWM3_MAXSTEP)
  {
    Pwm_cn[2]++;
    if(Pwm_cn[2] <= Pwm_dc[2])
      LED2_ON;
    else
      LED2_OFF;
  }  
  else
  {
   /*
    *  Expired counter
    *  Reload counters - force starting output
    */   
    Pwm_cn[2] = 0;
    LED2_OFF;   
  }  

}

/*
 *  This code is documented using DoxyGen 
 *  (http://www.stack.nl/~dimitri/doxygen/index.html)
 */



