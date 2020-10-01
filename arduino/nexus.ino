#include <math.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#define pin_A1 3 // Энкодер пин A
#define pin_B1  2 // Энкодер пин B
#define pin_A2 19 // Энкодер пин A
#define pin_B2  18 // Энкодер пин B

#define WHR 0.072 //m
#define WHEEL_LENGTH 2*PI*WHR
#define WHEEL_SEP 0.3

#define ENDSTOP_LEFT A2
#define ENDSTOP_CENTER A0
#define ENDSTOP_RIGHT A4

volatile long long Position_enc1 = 0;
volatile long long Position_enc2 = 0;
float rwheel_dist=0,lwheel_dist=0;
//----------------------------------------------

byte ena = 9;
byte in1 = 7;
byte in2 = 8;
byte ena1 = 6;
byte in3 = 4;
byte in4 = 5;

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0;
double vy = 0;
double vth = 0;
int pid(float set_speed,float pv_speed,float kp,float ki,float kd);

void init_motors()
{
    pinMode( ena, OUTPUT);
    pinMode( in1, OUTPUT);
    pinMode( in2, OUTPUT);
    pinMode( ena1, OUTPUT);
    pinMode( in3, OUTPUT);
    pinMode( in4, OUTPUT);
    pinMode(pin_A1, INPUT);
    pinMode(pin_B1, INPUT);
    pinMode(pin_A2, INPUT);
    pinMode(pin_B2, INPUT);
    pinMode(ENDSTOP_LEFT, INPUT);
    pinMode(ENDSTOP_CENTER, INPUT);
    pinMode(ENDSTOP_RIGHT, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin_A1), Encoder1_Rotate, RISING);
    attachInterrupt(digitalPinToInterrupt(pin_A2), Encoder2_Rotate, RISING);
}
  

ros::NodeHandle nh;



float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

void onTwist(const geometry_msgs::Twist &msg)
{
  float l = max(min((msg.linear.x - msg.angular.z) / 2, 1.0f), -1.0f);
  //*WHEEL_LENGTH/PI;
  float r = max(min((msg.linear.x + msg.angular.z) / 2, 1.0f), -1.0f);
  uint16_t lPwm = pid(l,lwheel_dist,1,1,1);
  uint16_t rPwm = pid(r,rwheel_dist,1,1,1);
  

set_rspd(rPwm);
set_lspd(lPwm);
}

//-----------------------------------------------

void Encoder1_Rotate() {
  //RIGHT
  if (digitalRead(pin_A1) == digitalRead(pin_B1)) {
    Position_enc1++;
  } else {
    Position_enc1--;
  }
}

void Encoder2_Rotate() {
  //LEFT
  if (digitalRead(pin_A2) == digitalRead(pin_B2)) {
    Position_enc2++;
  } else {
    Position_enc2--;
  }
}


void set_rspd(int pts)
{   if(pts > 0)
{analogWrite( ena, abs(pts) );
    digitalWrite( in1, LOW );
    digitalWrite( in2, HIGH );
    
}
else{
   analogWrite( ena, abs(pts) );
    digitalWrite( in1, HIGH );
    digitalWrite( in2, LOW );
   
    }
  
}

void set_lspd( int pts)
{
if(pts > 0)
{analogWrite( ena1, abs(pts) );
    digitalWrite( in3, HIGH );
    digitalWrite( in4, LOW );
    
}
else{
   analogWrite( ena1, abs(pts) );
    digitalWrite( in3, LOW );
    digitalWrite( in4, HIGH );
   
    }
   
 
}

std_msgs::Int16 msg;
nav_msgs::Odometry odom;
geometry_msgs::Twist debug_msg;
ros::Publisher r_endstop("r_endstop", &msg);
ros::Publisher l_endstop("l_endstop", &msg);
ros::Publisher c_endstop("c_endstop", &msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", onTwist );

ros::Publisher odom_pub("odom",&odom);

ros::Publisher deb_pub("debug",&debug_msg);
tf::TransformBroadcaster odom_broadcaster;
ros::Time current_time = nh.now();
ros::Time last_time = nh.now();


void setup()
{  
  nh.initNode();
  nh.advertise(r_endstop);
  nh.advertise(l_endstop);
  nh.advertise(c_endstop);
  nh.advertise(deb_pub);
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);
  init_motors();


}

int st=millis();
volatile long long old_ticksl=0,old_ticksr=0;
void loop()
{

    if(millis()-st>10)
    {
      lwheel_dist=(Position_enc2-old_ticksl)/ (768*0.01) * WHR;
      rwheel_dist=(Position_enc1-old_ticksr)/ (768*0.01) * WHR;
      st=millis();
      noInterrupts();
      debug_msg.linear.y = lwheel_dist;
      debug_msg.linear.z = rwheel_dist;
      debug_msg.angular.x = Position_enc2;
      debug_msg.angular.y = old_ticksl;
      
      old_ticksl = Position_enc2;
      //Position_enc2 = 0;
      old_ticksr = Position_enc1;
      //Position_enc1 = 0;
      interrupts();
      deb_pub.publish(&debug_msg);
    }
    nh.spinOnce();

    /*
    double dt = (current_time.toSec() - last_time.toSec());
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;
    
    x += delta_x;
    y += delta_y;
    th += delta_th;
    
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom_pub.publish(&odom);

    last_time = current_time;
    */
   msg.data = digitalRead(ENDSTOP_LEFT);
   l_endstop.publish( &msg );
   msg.data = digitalRead(ENDSTOP_CENTER);
   c_endstop.publish( &msg );
   msg.data = digitalRead(ENDSTOP_RIGHT);
   r_endstop.publish( &msg );
   delay(10);
}
int pid(float set_speed,float pv_speed,float kp,float ki,float kd)
{
  float error = set_speed - pv_speed;
  float prev_error=0, sum_error=0;
  int pwm_pulse = error*kp + sum_error*ki + (error - prev_error)*kd;
  

prev_error = error;

sum_error += error;

if (sum_error >4000) sum_error = 4000;

if (sum_error <-4000) sum_error = -4000;
debug_msg.linear.x = pwm_pulse;
debug_msg.angular.z = pv_speed;
deb_pub.publish(&debug_msg);
return pwm_pulse;
}
