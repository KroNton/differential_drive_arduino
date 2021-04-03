/* 
 * rosserial Ultrasound Example
 * 
 * This example is for the Maxbotix Ultrasound rangers.
 */

#include <ros.h>

#include <std_msgs/Int16.h>

#define RH_ENCODER_A  18
#define RH_ENCODER_B 19
#define LH_ENCODER_A 2
#define LH_ENCODER_B 8


ros::NodeHandle  nh;

std_msgs::Int16 len_msg;
std_msgs::Int16 ren_msg;
ros::Publisher pub_lencoder( "lwheel", &len_msg);
ros::Publisher pub_rencoder( "rwheel", &ren_msg);

volatile  long leftCounter = 0;
volatile  long rightCounter = 0;




void setup()
{
  nh.initNode();
  nh.advertise(pub_lencoder);
  nh.advertise(pub_rencoder);


  pinMode(LH_ENCODER_A, INPUT);  //Interrupts pin declaration
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  

  attachInterrupt(digitalPinToInterrupt(2), leftEncoderISR, CHANGE);   //attach interrupt fn
  attachInterrupt(digitalPinToInterrupt(18), rightEncoderISR, CHANGE);// relates the interrupt no.
  // the ISR Function and the Interrupt Mode

}




void loop()
{
  
    len_msg.data = leftCounter;
    ren_msg.data = rightCounter;
    
    pub_lencoder.publish( &len_msg );
    pub_rencoder.publish( &ren_msg );
  
  nh.spinOnce();
}


void leftEncoderISR()   //ISR for the Left Motor encoder
{         
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCounter++;    //if output A is High and Output B is low this means forward
    } else {
      leftCounter--;   // else then the motor is rotating backward so it subtracts the counter 
    }
    
  }
} 

void rightEncoderISR() 
{
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCounter++;
    } else {
      rightCounter--;
    }
  } 
}
