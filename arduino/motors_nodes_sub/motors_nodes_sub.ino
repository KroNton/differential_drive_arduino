/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Float32.h>

#define IN1 7
#define IN2 5

#define ENA 4
#define ENB 6

ros::NodeHandle  nh1;




void lmotor_Cb( const std_msgs::Float32& lm){
  
  left_motor_control(lm.data);
}

void rmotor_Cb( const std_msgs::Float32& rm){
  
  right_motor_control(rm.data);
}



ros::Subscriber<std_msgs::Float32> subl("lmotor_cmd", &lmotor_Cb );
ros::Subscriber<std_msgs::Float32> subr("rmotor_cmd", &rmotor_Cb );

void setup()
{ 
  //motorA
  pinMode(IN1, OUTPUT);
  pinMode(ENA, OUTPUT);
  //motorB
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  nh1.initNode();
  nh1.subscribe(subl);
  nh1.subscribe(subr);
  
}

void loop()
{  
  nh1.spinOnce();
  
  delay(1);
}


void left_motor_control(float vel1){
  
 //move the wheel Forward
  if(vel1> 0.0){
    digitalWrite(IN1,1);
    
    analogWrite(ENA,vel1);
    
    }
    
   //move the wheel backward
   
  else if(vel1< 0.0){
    
    digitalWrite(IN1,0);
    vel1=abs(vel1);
    analogWrite(ENA,vel1);
    }

   else if(vel1== 0) {
        
    analogWrite(ENA,0);
      }

    
  }



  void right_motor_control(float vel2){
  
 //move the wheel Forward
  if(vel2> 0.0){
    digitalWrite(IN2,1);
    
    analogWrite(ENB,vel2);
    
    }
    
   //move the wheel backward
   
  else if(vel2< 0.0){
    
    digitalWrite(IN2,0);
    vel2=abs(vel2);
    analogWrite(ENB,vel2);
    }

   else if(vel2== 0) {
        
    analogWrite(ENB,0);
      }

    
  }
