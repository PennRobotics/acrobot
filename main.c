/*
 * Acrobot
 *
 *  Created: 10/29/2015 4:25:37 PM
 *   Author: stevenem
 * Modified: brwr
 */ 

//TODO 
// First method of seeking stability: [values shown not accurate]
//      PWM  70/255 ( 27% duty) and   set(R)   set(L) at    0 degrees
//      PWM 102/255 ( 40% duty) and clear(R)   set(L) at  -10 degrees
//      PWM 102/255 ( 40% duty) and   set(R) clear(L) at  +10 degrees

//TODO 
// Setup:
//  Attempting to hold wheels stationary and quickly stabilize vertically
// Problem:
//  Quick enough to move upright.
//  Overshoots. Doesn't hold at vertical.
// Potential solutions:
//  Widen stop gap.
//  Apply PWM 255/255 (100% duty) at stop gap.
//  Apply PWM 255/255 on inactive motor; adjust PWM down on active motor.
//    (This always applies full brake on one while selectively releasing other.)
//  Adjust proportional gain.
//  Adjust duty offset so motor barely but fully moves vertical from horizontal.

#include "m_general.h"
#include "m_imu.h"
#include "m_usb.h"
#include "m_bus.h"
#include <math.h>

#define BETA .988 /*low-pass filter constant*/
#define NUM_OF_CYCLES_PER_MOTOR_REFRESH 3

signed int acc[3]; //incoming accelerometer data array
signed int gyro[3]; //gyro array

float filteredAngle = 0; //calculated angle in degrees
float filteredAngleVel = 0;
int angSpeed = 0; //TODO

int motOut; //motor output
float gainP = 22.0; //proportional gain
float gainD = 33; //derivative gain
float gainI = 0; //integral gain

int n = 0; //counter for averaging function

void init(void); //setup function
void ang(void); //calculate current angle
void motorUpdate(void); //motor update

float currentAngle;
float currentAngleVel;
float previousAngleVel = 0;
float currentAngleVelInt;
float filteredAngleA;
float filteredAngleG;
float filteredAngle;

int main(void)
{
  init(); 

  while(1){	}
}

void init(void){
  m_clockdivide(0); //16 MHz
  m_usb_init(); //enable usb comm
  m_imu_init(0, 0); //enable IMU

  //TIMER 3

  //TODO Check timer settings
  clear(TCCR3B,CS32); // system clock/1
  clear(TCCR3B,CS31);
  set(TCCR3B,CS30);


  clear(TCCR3B,WGM33); //up to OCR3A and reset
  set(TCCR3B,WGM32);
  clear(TCCR3A,WGM31);
  clear(TCCR3A,WGM30);

  OCR3A = 16000; //1000 hz

  set(TIMSK3,OCIE3A); //interrupt when counter hits OCR3A 
  sei(); //enable global interrupts

  set(DDRE,6);
  set(PORTE,6);
//  set(DDRB,4); //B4 and 5 to output
//  set(DDRB,5); //motor in pins

  //TIMER 4

  OCR4C = 255;

  //set(TCCR4E,ENHC4);//enhanced pwm

  // Prescaler
  clear(TCCR4B,CS43);
  set(TCCR4B,CS42);
  clear(TCCR4B,CS41);
  set(TCCR4B,CS40);

  // Waveform (up and reset; sawtooth-style)
  clear(TCCR4D,WGM41);
  clear(TCCR4D,WGM40);

  set(TCCR4A,PWM4A); //low at OCR4A, high at reset
  set(TCCR4A,COM4A1);
  clear(TCCR4A,COM4A0);

  set(TCCR4A,PWM4B); //low at OCR4B, high at reset
  set(TCCR4A,COM4B1);
  clear(TCCR4A,COM4B0);

  set(DDRC,7); //PWM output OC4A
  set(DDRB,6); //PWM output OC4B

  /*set(ADMUX,REFS0); //Vcc ref voltage

    set(ADCSRA,ADPS2); // prescale clock to 1/128 system (125 kHz)
    set(ADCSRA,ADPS1);
    set(ADCSRA,ADPS0);

    set(DIDR0, ADC0D); //disable digital inputs

    set(ADCSRA, ADIE);//enable interrupt

    set(ADCSRA, ADATE); //free-running mode

    set(ADCSRA, ADEN); //enable ADC

    set(ADCSRA,ADSC); //begin conversion
    */
}


void motorUpdate(void){

//  motOut = fmin(255, gainP * fabs((double)filteredAngle) - gainD * angSpeed + 70);

//  OCR4A = (unsigned char) motOut;

  if(filteredAngle<=0) { //if acceleration detection in positive direction
    OCR4A = 255 + fmax(-252, filteredAngle); //TODO ...minus PWM_OFFSET
    OCR4B = 255;
//    motOut = fmin(255, gainP * fabs((double)angle) - gainD * angSpeed + 70);
    m_green(OFF);
//    clear(PORTB,4);
//    set  (PORTB,5);
  } else if(filteredAngle>0) { //acceleration detected in negative direction
    OCR4A = 255;
    OCR4B = 255 - fmin(252, filteredAngle); //TODO ...minus PWM_OFFSET
//    motOut = fmin(255, gainP * fabs((double)angle) + gainD * angSpeed + 70);
    m_green(ON);
//    set  (PORTB,4);
//    clear(PORTB,5);
  } else {
    OCR4A = 255;
    OCR4B = 255;
    m_green(TOGGLE);
//    set  (PORTB,4);
//    set  (PORTB,5);
  }

//  n = 0;

}

ISR(TIMER3_COMPA_vect){
//  m_red(TOGGLE);

  m_imu_accel((signed int*) acc); //read IMU every millisecond
  m_imu_gyro ((signed int*) gyro);

  n++;


  // DELTA_i_av = BETA * DELTA_i-1_av + (1-BETA)(-a_i/g)
  // This function differs in that it uses
  //  atan2 of two accel values to find angle.
  currentAngle       = (6 + 57.3*atan2(-acc[2],-acc[0]));
  filteredAngleA = BETA * filteredAngleA + (1-BETA) * currentAngle;

  // PSI_i = PSI_i-1 + OMEGA_I dt
  // PSI_i_av = BETA * PSI_i-1_av + BETA * (PSI_i - PSI_i-1)
  currentAngleVel    = (250*(double)gyro[1])/16384.0;
//  previousAngleVelInt   = currentAngleVelInt;
//  currentAngleVelInt = currentAngleVelInt + 0.001 * currentAngleVel;
  filteredAngleG = BETA * filteredAngleG + BETA * (0.001 * currentAngleVel);
//  filteredAngleG = BETA * filteredAngleG + BETA * (currentAngleVelInt - previousAngleVelInt);

  filteredAngle = gainP * filteredAngleA + gainD * filteredAngleG;

  if (n >= NUM_OF_CYCLES_PER_MOTOR_REFRESH) {
    n = 0;
    motorUpdate();

    //USB Accelerometer debugging
    m_usb_tx_string("\n");
//    m_usb_tx_string(  "ACC0   ");
//    m_usb_tx_int(acc[0]); 
//    m_usb_tx_string("\tACC1   ");
//    m_usb_tx_int(acc[1]); 
//    m_usb_tx_string("\tACC2   ");
//    m_usb_tx_int(acc[2]); 
//    m_usb_tx_string(  "GY 0   ");
//    m_usb_tx_int(gyro[0]); 
//    m_usb_tx_string("\tGY 1   ");
//    m_usb_tx_int(gyro[1]); 
//    m_usb_tx_string("\tGY 2   ");
//    m_usb_tx_int(gyro[2]); 
//    m_usb_tx_string("\tPWM<<  ");
//    m_usb_tx_int((int)OCR4A); 
//    m_usb_tx_string("\tPWM>>  ");
//    m_usb_tx_int((int)OCR4B); 
    m_usb_tx_string("\tANGLE  ");
    m_usb_tx_int(filteredAngle); 
//    m_usb_tx_string("\tANG V  ");
//    m_usb_tx_int(currentAngleVel); 
//    m_usb_tx_string("\tINT    ");
//    m_usb_tx_int(currentAngleVelInt); 
    m_usb_tx_string("\tG COMP ");
    m_usb_tx_int(filteredAngleG); 
    m_usb_tx_string("\tA COMP ");
    m_usb_tx_int(filteredAngleA); 
  }

//Old code:
//  angle = beta*(angle + (samptime/1000.0)*(float)gyro[0]*conv)+ (1-beta)*(double)deg*asin((double)((double)accZ/16384.0));
//  angSpeed = (angle - angleOld)/samptime;

}
