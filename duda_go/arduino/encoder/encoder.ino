#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

std_msgs::Int16 int16;
ros::Publisher lwheel("lwheel", &int16);
ros::Publisher rwheel("rwheel", &int16);

int counter ;
int state ; 
int lastState ; 

int counter2 ;
int state2 ; 
int lastState2 ; 

void Lcallback ( const std_msgs::Int16& msg){
 int16.data = msg.data;
 lwheel.publish( &int16 );
}

void Rcallback ( const std_msgs::Int16& msg){
 int16.data = msg.data;
 rwheel.publish( &int16 );
}
 
void setup()
{
    nh.initNode();
    nh.advertise(lwheel);
    nh.advertise(rwheel);
    Serial.begin(2000000);
}

void loop()
{
    if (analogRead(7) > 670) {
      state = 1 ; 
      //strip.setPixelColor(0 , strip.Color(150,150,150)) ; 
    } else {
      state = 0 ; 
      //strip.setPixelColor(0 , strip.Color(0,0,0)) ; 
    }

    //strip.show() ;

    if (analogRead(1) > 670) {
      state2 = 1 ; 
      //strip.setPixelColor(1 , strip.Color(150,150,150)) ; 
    } else {
      state2 = 0 ; 
      //strip.setPixelColor(1 , strip.Color(0,0,0)) ; 
    }

    //strip.show() ;


  if ( state != lastState) { 
    if(state == HIGH) {
      counter++ ; 
      int16.data = counter;
      lwheel.publish( &int16 );
    }
  }

  lastState = state ;

  if ( state2 != lastState2) { 
    if(state2 == HIGH) {
      counter2++ ; 
      int16.data = counter2;
      rwheel.publish( &int16 );
    }
  }
 
  lastState2 = state2 ; 
  
  nh.spinOnce();
}
