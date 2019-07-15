/**********************************************************/
/* Demo program for:                                      */
/*    Board: SHIELD-EKG/EMG + Olimexino328                */
/*  Manufacture: OLIMEX                                   */
/*  COPYRIGHT (C) 2012                                    */
/*  Designed by:  Penko Todorov Bozhkov                   */
/*   Module Name:   Sketch                                */
/*   File   Name:   ShieldEkgEmgDemo.ino                  */
/*   Revision:  Rev.A                                     */
/*    -> Added is suppport for all Arduino boards.        */
/*       This code could be recompiled for all of them!   */
/*   Date: 19.12.2012                                     */
/*   Built with Arduino C/C++ Compiler, version: 1.0.3    */
/**********************************************************/
/**********************************************************
  Purpose of this programme is to give you an easy way to
  connect Olimexino328 to ElectricGuru(TM), see:
  https://www.olimex.com/Products/EEG/OpenEEG/EEG-SMT/resources/ElecGuru40.zip
  where you'll be able to observe yours own EKG or EMG signal.
  It is based on:
***********************************************************
  ModularEEG firmware for one-way transmission, v0.5.4-p2
  Copyright (c) 2002-2003, Joerg Hansmann, Jim Peters, Andreas Robinson
  License: GNU General Public License (GPL) v2
***********************************************************
  For proper communication packet format given below have to be supported:
  ///////////////////////////////////////////////
  ////////// Packet Format Version 2 ////////////
  ///////////////////////////////////////////////
  // 17-byte packets are transmitted from Olimexino328 at 256Hz,
  // using 1 start bit, 8 data bits, 1 stop bit, no parity, 57600 bits per second.

  // Minimial transmission speed is 256Hz * sizeof(Olimexino328_packet) * 10 = 43520 bps.

  struct Olimexino328_packet
  {
  uint8_t	sync0;		// = 0xa5
  uint8_t	sync1;		// = 0x5a
  uint8_t	version;	// = 2 (packet version)
  uint8_t	count;		// packet counter. Increases by 1 each packet.
  uint16_t	data[6];	// 10-bit sample (= 0 - 1023) in big endian (Motorola) format.
  uint8_t	switches;	// State of PD5 to PD2, in bits 3 to 0.
  };
*/
/**********************************************************/


//#include <compat/deprecated.h>
#include <FlexiTimer2.h>
//http://www.arduino.cc/playground/Main/FlexiTimer2


//#include <MuscleControl.h>

// Arduino pins for the shift register
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

// 8-bit bus after the 74HC595 shift register
// (not Arduino pins)
// These are used to set the direction of the bridge driver.
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6

// Arduino pins for the PWM signals.
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5

// Codes for the motor function.
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4


// All definitions
#define NUMCHANNELS 6
#define HEADERLEN 4
#define PACKETLEN (NUMCHANNELS * 2 + HEADERLEN + 1)
#define SAMPFREQ 256                      // ADC sampling rate 256
#define TIMER2VAL (1024/(SAMPFREQ))       // Set 256Hz sampling frequency                    
#define LED1  13
#define CAL_SIG 9
const int num_read = 26;
// Global constants and variables
volatile unsigned char TXBuf[PACKETLEN];  //The transmission packet
volatile unsigned char TXIndex;           //Next byte to write in the transmission packet.
volatile unsigned char CurrentCh;         //Current channel being sampled.
volatile unsigned char counter = 0;	  //Additional divider used to generate CAL_SIG
volatile unsigned int ADC_Value[6] = {0, 0, 0, 0, 0, 0};	 //ADC current value
float Data[num_read][2]; //Data circular buffer
float RMS[3][2];
float F[2][2];
int Data_p = 0, one23 = 0;
float B_f[3] = { -0.0214, 0.9572, -0.0214};
byte motor_speed = 0;


//~~~~~~~~~~
// Functions
//~~~~~~~~~~

/****************************************************/
/*  Function name: Toggle_LED1                      */
/*  Parameters                                      */
/*    Input   :  No	                            */
/*    Output  :  No                                 */
/*    Action: Switches-over LED1.                   */
/****************************************************/
void Toggle_LED1(void) {

  if ((digitalRead(LED1)) == HIGH) {
    digitalWrite(LED1, LOW);
  }
  else {
    digitalWrite(LED1, HIGH);
  }

}


/****************************************************/
/*  Function name: toggle_GAL_SIG                   */
/*  Parameters                                      */
/*    Input   :  No	                            */
/*    Output  :  No                                 */
/*    Action: Switches-over GAL_SIG.                */
/****************************************************/
void toggle_GAL_SIG(void) {

  if (digitalRead(CAL_SIG) == HIGH) {
    digitalWrite(CAL_SIG, LOW);
  }
  else {
    digitalWrite(CAL_SIG, HIGH);
  }

}


/****************************************************/
/*  Function name: setup                            */
/*  Parameters                                      */
/*    Input   :  No	                            */
/*    Output  :  No                                 */
/*    Action: Initializes all peripherals           */
/****************************************************/
void setup() {


  // Serial Port
  //Serial.begin(57600);
  Serial.begin(115200);
  establishContact();
  //Set speed to 57600 bps

  //Serial.println("before interupts");
  delay(100);
  noInterrupts();  // Disable all interrupts before initialization

  for (int i = 1; i < num_read; i++)
  { Data[i][1] = 0;
    Data[i][2] = 0;
  }


  // LED1
  pinMode(LED1, OUTPUT);  //Setup LED1 direction
  digitalWrite(LED1, LOW); //Setup LED1 state
  pinMode(CAL_SIG, OUTPUT);

  //Write packet header and footer
  TXBuf[0] = 0xa5;    //Sync 0
  TXBuf[1] = 0x5a;    //Sync 1
  TXBuf[2] = 2;       //Protocol version
  TXBuf[3] = 0;       //Packet counter
  TXBuf[4] = 0x02;    //CH1 High Byte
  TXBuf[5] = 0x00;    //CH1 Low Byte
  TXBuf[6] = 0x02;    //CH2 High Byte
  TXBuf[7] = 0x00;    //CH2 Low Byte
  TXBuf[8] = 0x02;    //CH3 High Byte
  TXBuf[9] = 0x00;    //CH3 Low Byte
  TXBuf[10] = 0x02;   //CH4 High Byte
  TXBuf[11] = 0x00;   //CH4 Low Byte
  TXBuf[12] = 0x02;   //CH5 High Byte
  TXBuf[13] = 0x00;   //CH5 Low Byte
  TXBuf[14] = 0x02;   //CH6 High Byte
  TXBuf[15] = 0x00;   //CH6 Low Byte
  TXBuf[2 * NUMCHANNELS + HEADERLEN] =  0x01;	// Switches state

  // Timer2
  // Timer2 is used to setup the analag channels sampling frequency and packet update.
  // Whenever interrupt occures, the current read packet is sent to the PC
  // In addition the CAL_SIG is generated as well, so Timer1 is not required in this case!
  FlexiTimer2::set(TIMER2VAL, Timer2_Overflow_ISR);
  FlexiTimer2::start();


  // MCU sleep mode = idle.
  //outb(MCUCR,(inp(MCUCR) | (1<<SE)) & (~(1<<SM0) | ~(1<<SM1) | ~(1<<SM2)));

  interrupts();  // Enable all interrupts after initialization has been completed

  //delay(100);
  //Serial.println("HELLO");
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
  }
}




/****************************************************/
/*  Function name: Timer2_Overflow_ISR              */
/*  Parameters                                      */
/*    Input   :  No	                            */
/*    Output  :  No                                 */
/*    Action: Determines ADC sampling frequency.    */
/****************************************************/
void Timer2_Overflow_ISR()
{
  // Toggle LED1 with ADC sampling frequency /2
  Toggle_LED1();
  //Serial.write("RMS1");

  //Read the 6 ADC inputs and store current values in Packet
  for (CurrentCh = 0; CurrentCh < 6; CurrentCh++) {
    ADC_Value[CurrentCh] = analogRead(CurrentCh);
    TXBuf[((2 * CurrentCh) + HEADERLEN)] = ((unsigned char)((ADC_Value[CurrentCh] & 0xFF00) >> 8));	// Write High Byte
    TXBuf[((2 * CurrentCh) + HEADERLEN + 1)] = ((unsigned char)(ADC_Value[CurrentCh] & 0x00FF));	// Write Low Byte
  }

  
  
  // Send Packet
  for (TXIndex = 0; TXIndex < 17; TXIndex++) {
    Serial.write(TXBuf[TXIndex]);
  }

 
  

  // Increment the packet counter
  TXBuf[3]++;
  Data_p++;
  if (Data_p > num_read) {
    Data_p = 0;
 
  };


  {
  }
  // Generate the CAL_SIGnal
  counter++;		// increment the devider counter
  if (counter == 12) {	// 250/12/2 = 10.4Hz ->Toggle frequency
    counter = 0;
    toggle_GAL_SIG();	// Generate CAL signal with frequ ~10Hz
  }


  
}


/****************************************************/
/*  Function name: loop                             */
/*  Parameters                                      */
/*    Input   :  No	                            */
/*    Output  :  No                                 */
/*    Action: Puts MCU into sleep mode.             */
/****************************************************/
void loop() {
  // motor.run(FORWARD);
  // motor.setSpeed(1);
  // delay(10);
  //__asm__ __volatile__ ("sleep");

    if (Serial.available()>0  )
  {
        motor_speed=Serial.read();
        //motor_speed =255;
        motor(1,FORWARD,motor_speed);
       
  }


}

void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {
    switch (nMotor)
    {
      case 1:
        motorA   = MOTOR1_A;
        motorB   = MOTOR1_B;
        break;
      case 2:
        motorA   = MOTOR2_A;
        motorB   = MOTOR2_B;
        break;
      case 3:
        motorA   = MOTOR3_A;
        motorB   = MOTOR3_B;
        break;
      case 4:
        motorA   = MOTOR4_A;
        motorB   = MOTOR4_B;
        break;
      default:
        break;
    }

    switch (command)
    {
      case FORWARD:
        motor_output (motorA, HIGH, speed);
        motor_output (motorB, LOW, -1);     // -1: no PWM set
        break;
      case BACKWARD:
        motor_output (motorA, LOW, speed);
        motor_output (motorB, HIGH, -1);    // -1: no PWM set
        break;
      case BRAKE:
        // The AdaFruit library didn't implement a brake.
        // The L293D motor driver ic doesn't have a good
        // brake anyway.
        // It uses transistors inside, and not mosfets.
        // Some use a software break, by using a short
        // reverse voltage.
        // This brake will try to brake, by enabling
        // the output and by pulling both outputs to ground.
        // But it isn't a good break.
        motor_output (motorA, LOW, 255); // 255: fully on.
        motor_output (motorB, LOW, -1);  // -1: no PWM set
        break;
      case RELEASE:
        motor_output (motorA, LOW, 0);  // 0: output floating.
        motor_output (motorB, LOW, -1); // -1: no PWM set
        break;
      default:
        break;
    }
  }
}


// ---------------------------------
// motor_output
//
// The function motor_ouput uses the motor driver to
// drive normal outputs like lights, relays, solenoids,
// DC motors (but not in reverse).
//
// It is also used as an internal helper function
// for the motor() function.
//
// The high_low variable should be set 'HIGH'
// to drive lights, etc.
// It can be set 'LOW', to switch it off,
// but also a 'speed' of 0 will switch it off.
//
// The 'speed' sets the PWM for 0...255, and is for
// both pins of the motor output.
//   For example, if motor 3 side 'A' is used to for a
//   dimmed light at 50% (speed is 128), also the
//   motor 3 side 'B' output will be dimmed for 50%.
// Set to 0 for completelty off (high impedance).
// Set to 255 for fully on.
// Special settings for the PWM speed:
//    Set to -1 for not setting the PWM at all.
//
void motor_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
    case MOTOR1_A:
    case MOTOR1_B:
      motorPWM = MOTOR1_PWM;
      break;
    case MOTOR2_A:
    case MOTOR2_B:
      motorPWM = MOTOR2_PWM;
      break;
    case MOTOR3_A:
    case MOTOR3_B:
      motorPWM = MOTOR3_PWM;
      break;
    case MOTOR4_A:
    case MOTOR4_B:
      motorPWM = MOTOR4_PWM;
      break;
    default:
      // Use speed as error flag, -3333 = invalid output.
      speed = -3333;
      break;
  }

  if (speed != -3333)
  {
    // Set the direction with the shift register
    // on the MotorShield, even if the speed = -1.
    // In that case the direction will be set, but
    // not the PWM.
    shiftWrite(output, high_low);

    // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)
    {
      analogWrite(motorPWM, speed);
    }
  }
}


// ---------------------------------
// shiftWrite
//
// The parameters are just like digitalWrite().
//
// The output is the pin 0...7 (the pin behind
// the shift register).
// The second parameter is HIGH or LOW.
//
// There is no initialization function.
// Initialization is automatically done at the first
// time it is used.
//
void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  // Do the initialization on the fly,
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    // start with all outputs (of the shift register) low
    latch_copy = 0;

    shift_register_initialized = true;
  }

  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);

  // Use the default Arduino 'shiftOut()' function to
  // shift the bits with the MOTORCLK as clock pulse.
  // The 74HC595 shiftregister wants the MSB first.
  // After that, generate a latch pulse with MOTORLATCH.
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}
