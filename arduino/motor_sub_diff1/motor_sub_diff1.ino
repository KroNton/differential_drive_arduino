/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Float32.h>

#define IN1 7
#define IN2 8

#define ENA 3

ros::NodeHandle  nh1;




void lmotor_Cb( const std_msgs::Float32& lm){
  
  left_motor_control(lm.data);
}


ros::Subscriber<std_msgs::Float32> subl("lwheel_vtarget", &lmotor_Cb );

void setup()
{ 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  
  pinMode(ENA, OUTPUT);
  
  nh1.initNode();
  nh1.subscribe(subl);
  
}

void loop()
{  
  nh1.spinOnce();
  
  delay(1);
}


void left_motor_control(float vel1){
  
 // mapping speed value
 
float vel_1=abs(vel1);
float left_motor_speed= mapfloat(vel_1, 0.0, 1.0, 0.0, 255.0);


//scalling speed value
//float  left_motor_speed=abs(vel1)*255;


  //move the wheel forward
  
  if(vel1> 0.0){
    digitalWrite(IN1,1);
    digitalWrite(IN2,0);
    analogWrite(ENA,left_motor_speed);
    
    }
    
   //move the wheel backward
   
  else if(vel1< 0.0){
    
    digitalWrite(IN1,0);
    digitalWrite(IN2,1);
    analogWrite(ENA,left_motor_speed);
    }

    else if(vel1== 0) {
      
    digitalWrite(IN1,0);
    digitalWrite(IN2,0);     
    analogWrite(ENA,0);
      }

    
  }

  float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
