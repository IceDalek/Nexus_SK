#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <DynamixelWorkbench.h>

#define DEVICE_NAME ""

#define BAUDRATE  1000000
#define LDXL_ID    2
#define RDXL_ID    1

DynamixelWorkbench dxl_wb;

volatile long Position_encL = 0;
volatile long Position_encR = 0;
uint16_t model_number = 0;
uint8_t rdxl_id = RDXL_ID;
uint8_t ldxl_id = LDXL_ID;
//----------------------------------------------

ros::NodeHandle nh;

//-----------------------------------------------

void EncoderL_Read() {
  static int32_t old_pos=0,n=0;
  int32_t pos;
dxl_wb.getPresentPositionData(ldxl_id,&pos);
if((old_pos>4000)&&(Position_encL<50))
  n++;
  Position_encL=4096*n+pos;
  old_pos=pos;
}

void EncoderR_Read() {
 static int32_t old_pos=0,n=0;
  int32_t pos;
dxl_wb.getPresentPositionData(rdxl_id,&pos);
if((old_pos>4000)&&(Position_encL<100))
  n++;
  Position_encR=4096*n+pos;
  old_pos=pos;
}
void read_enc()
{
EncoderR_Read();
EncoderL_Read();
}
void set_rspd( const std_msgs::Float32& msg)
{
  dxl_wb.goalVelocity(rdxl_id, (int32_t)msg.data);
  /* value: between -1023 and 1023. 285?*/
}

void set_lspd( const std_msgs::Float32& msg)
{
  
dxl_wb.goalVelocity(ldxl_id, (int32_t)msg.data);
  /* value: between -1023 and 1023. 285?*/ 
}

std_msgs::Int16 msg;
ros::Publisher lwpub("lwheel", &msg);
ros::Publisher rwpub("rwheel", &msg);
ros::Subscriber<std_msgs::Float32> rmotor("rmotor_cmd", set_rspd );
ros::Subscriber<std_msgs::Float32> lmotor("lmotor_cmd", set_lspd );

void setup()
{  
  nh.initNode();
  nh.advertise(lwpub);
  nh.advertise(rwpub);
  nh.subscribe(rmotor);
  nh.subscribe(lmotor);
  
  bool result = dxl_wb.init(DEVICE_NAME, BAUDRATE);
  if (!result)
  {
    //send false to debug topic
  }
  else
  {
    dxl_wb.ping(rdxl_id, &model_number);
    dxl_wb.ping(ldxl_id, &model_number);
     //send ok to debug topic
  }

  dxl_wb.wheelMode(ldxl_id, 0);
  dxl_wb.wheelMode(rdxl_id, 0);
}


void loop()
{
  
 while (!nh.connected())
  {
    nh.spinOnce();
  }
  read_enc();
   msg.data = Position_encR;
   rwpub.publish( &msg );
   msg.data = Position_encL;
   lwpub.publish( &msg );
   nh.spinOnce();
   delay(10);
}
