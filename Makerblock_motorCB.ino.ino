/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    MeMegaPiDCMotorTest.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2016/05/17
 * @brief   Description: this file is sample code for MegaPi DC motor device.
 *
 * Function List:
 *    1. void MeMegaPiDCMotorTest::run(int16_t speed)
 *    2. void MeMegaPiDCMotorTest::stop(void)
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2016/05/17    1.0.0          build the new
 * </pre>
 */
#include "MeMegaPi.h"
//#include <MeMegaPiPro.h>

// for nexus
#include <math.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#define pin_A1 3 // Энкодер пин A
#define pin_B1  2 // Энкодер пин B
#define pin_A2 19 // Энкодер пин A
#define pin_B2  18 // Энкодер пин B

#define WHR 0.072 //m
#define CPR 770 
#define WHEEL_LENGTH 2*PI*WHR
#define WHEEL_SEP 0.3

volatile long Position_enc1 = 0;
volatile long Position_enc2 = 0;
float p1 =0;
float p2 =0;
float x=0,y=0;
//----------------------------------------------



ros::NodeHandle nh;



//for megapi
MeMegaPiDCMotor motor1(PORT1A);

MeMegaPiDCMotor motor2(PORT1B);

MeMegaPiDCMotor motor3(PORT2A);

MeMegaPiDCMotor motor4(PORT2B);

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

//-----------------------------------------------

void Encoder1_Rotate() {
  if (digitalRead(pin_A1) == digitalRead(pin_B1)) {
    Position_enc1++;
  } else {
    Position_enc1--;
  }
}

// for nexus
void Encoder2_Rotate() {
  if (digitalRead(pin_A2) == digitalRead(pin_B2)) {
    Position_enc2++;
  } else {
    Position_enc2--;
  }
}

void whcalk(int ticks,int ticks2)
{
     p1=ticks/CPR*WHEEL_LENGTH;
     p2=ticks2/CPR*WHEEL_LENGTH;
}

void set_rspd( const std_msgs::Float32& msg)
{
  
  motor2.run(int(msg.data)); /* value: between -255 and 255. */
}

void set_lspd( const std_msgs::Float32& msg)
{
  

  motor4.run(int(-msg.data)); /* value: between -255 and 255. */
   
 
}
void run_m(int data){
  motor4.run(data);
  }

std_msgs::Int16 msg;
ros::Publisher lwpub("lwheel", &msg);
ros::Publisher rwpub("rwheel", &msg);
ros::Subscriber<std_msgs::Float32> rmotor("rmotor_cmd", set_rspd );
ros::Subscriber<std_msgs::Float32> lmotor("lmotor_cmd", set_lspd );


uint8_t motorSpeed = 100;

void setup()
{  
  nh.initNode();
  nh.advertise(lwpub);
  nh.advertise(rwpub);
  nh.subscribe(rmotor);
  nh.subscribe(lmotor);
  
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  //Serial.begin(115200);

//set pwm 1khz
  TCCR1A = _BV(WGM10);//PIN12
  TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);
 
  TCCR2A = _BV(WGM21) | _BV(WGM20);//PIN8
  TCCR2B = _BV(CS22);

  TCCR3A = _BV(WGM30);//PIN9
  TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);

  TCCR4A = _BV(WGM40);//PIN5
  TCCR4B = _BV(CS41) | _BV(CS40) | _BV(WGM42);
}


void loop()
{/* value: between -255 and 255. */
  //motor2.run(-motorSpeed);
  //motor4.run(-motorSpeed);
//run_m(200);

  
  Encoder_1.loop();
  Encoder_2.loop();
     while (!nh.connected())
  {
    nh.spinOnce();
  }
  
   msg.data = Encoder_1.getCurPos();
   rwpub.publish( &msg );
   msg.data = Encoder_2.getCurPos();
   lwpub.publish( &msg );
   nh.spinOnce();
   delay(10);
 // Serial.print("Spped 1:");
 // Serial.print(Encoder_1.getCurPos());
 // Serial.pint(" ,Spped 2:");
 // Serial.println(Encoder_2.getCurPos());
}
