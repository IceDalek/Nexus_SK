

#include "ros/ros.h"
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <stdio.h>
using namespace std;
#include <unistd.h>
#include <termios.h>
#define max_linear  1.5
#define max_angular  1.5
char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}
struct Vel
{
	float x = 0;
	float z = 0;
};
void log_speed(Vel vel){
cout << "Current linear speed: " << vel.x << endl;
cout << "Current angular speed: " << vel.z << endl << endl << endl;
cout << " ----------W----------" << endl;
cout << " ----A-----------S----" << endl;
cout << " ----------D---------- " << endl;
cout << "w to increace linear velocity" << endl;
cout << "a to increace angular velocity" << endl;
cout << "d to increace angular velocity" << endl;
cout << "w to increace linear velocity" << endl;
cout << "x to force stop" << endl;
cout << "q to exit" << endl;
cout << "-----------------------------" << endl;

}

int main(int argc, char**argv){
ros::init(argc, argv, "publish_velocity");
ros::NodeHandle nh;

ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
geometry_msgs::Twist msg;
ros::Rate rate(10);

Vel vel; 
while(ros::ok()){
	char c;

	c = getch();
	    if (c == 'w'){
	    	if(vel.x > max_linear){
	    		vel.x = vel.x;
	    	}
	    	else{
	    	vel.x += 0.1;
	    	}
	    }
	    if (c == 's'){
	    	if(vel.x < -max_linear){
	    	vel.x = vel.x;
	    	}
	    else{
			vel.x -= 0.1;
	    	}
	    }

	    if (c == 'd'){
	    	if(vel.z < -max_angular){
	    		vel.z = vel.z;
	    	}
	    	else{
	    	vel.z -= 0.1;
	   		}
		}
	    if (c == 'a'){
	    	if(vel.z > max_angular){
	    		vel.z = vel.z;
	    	}
	    	else{
	    	vel.z += 0.1;
	    	}
	    }
	    if( c == 'x'){
	    	vel.x = 0;
	    	vel.z = 0;
	    }
	    if (c == 'q'){
	    	exit(0);
	    }
	    msg.linear.x = vel.x;
	    msg.angular.z = vel.z;
	    log_speed(vel);
	    
		pub.publish(msg);    
		
	}


}
