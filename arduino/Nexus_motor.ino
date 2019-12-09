/**
byte ena = 9;
byte in1 = 7;
byte in2 = 8;
byte ena1 = 6;
byte in3 = 4;
byte in4 = 5;
void setup() {

}
void loop() {
  //backward
   
    analogWrite( ena, 100 );
    analogWrite( ena1, 100 );
   
    digitalWrite( in1, HIGH );
    digitalWrite( in2, LOW );
    digitalWrite( in3, HIGH );
    digitalWrite( in4, LOW );
    /*
    delay(3000); // пауза 3сек
    // выставляем мощность на мотора А - 150 из 255
    analogWrite( ena1, 100 );
    analogWrite( ena, 100 );
    // режим мотора - вращение против часовой
    digitalWrite( in1, LOW );
    digitalWrite( in2, HIGH );
        digitalWrite( in3, LOW );
    digitalWrite( in4, HIGH );
    delay(3000); // пауза 3сек
    */


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

byte ena = 9;
byte in1 = 7;
byte in2 = 8;
byte ena1 = 6;
byte in3 = 4;
byte in4 = 5;
void init_motors()
{
    pinMode( ena, OUTPUT );
    pinMode( in1, OUTPUT );
    pinMode( in2, OUTPUT );
    pinMode( ena1, OUTPUT );
    pinMode( in3, OUTPUT );
    pinMode( in4, OUTPUT );
    pinMode(pin_A1, INPUT);
    pinMode(pin_B1, INPUT);
    pinMode(pin_A2, INPUT);
    pinMode(pin_B2, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin_A1), Encoder1_Rotate, RISING);
    attachInterrupt(digitalPinToInterrupt(pin_A2), Encoder2_Rotate, RISING);
}
  

ros::NodeHandle nh;



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
{   if(msg.data > 0)
{analogWrite( ena, abs(msg.data) );
    digitalWrite( in1, HIGH );
    digitalWrite( in2, LOW );
    
}
else{
   analogWrite( ena, abs(msg.data) );
    digitalWrite( in1, LOW );
    digitalWrite( in2, HIGH );
   
    }
  
}

void set_lspd( const std_msgs::Float32& msg)
{
  

if(msg.data > 0)
{analogWrite( ena1, abs(msg.data) );
    digitalWrite( in3, HIGH );
    digitalWrite( in4, LOW );
    
}
else{
   analogWrite( ena1, abs(msg.data) );
    digitalWrite( in3, LOW );
    digitalWrite( in4, HIGH );
   
    }
   
 
}
void run_m(int data){
 
   
  //  analogWrite( ena, 100 );
   // analogWrite( ena1, 100 );
   
    digitalWrite( in1, HIGH );
    digitalWrite( in2, LOW );
    digitalWrite( in3, HIGH );
    digitalWrite( in4, LOW );
  
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
  init_motors();
 // Serial.begin(115200);


}


void loop()
{

  
   msg.data = Position_enc1;
   rwpub.publish( &msg );
   msg.data = Position_enc2;
   lwpub.publish( &msg );
   //Serial.println(Position_enc2);
 //  run_m(100);
 //  delay(5000);
  // run_m(-100);
   nh.spinOnce();
   delay(10);

}
