#include <Streaming.h> //to print stuff easier
#include <AX12A.h>

#define DirectionPin (10u)
#define ID            (1u)
unsigned char ID1 = 1;
#define BaudRate   (1000000)

//This can be edited
    
int curr_servo_ang=0;
int prev_servo_ang=0;


void setup()
{
 Serial.begin(1000000);

 pinMode(13, OUTPUT);

 //-90-90 for upper servo, 0 degrees is upright
 ax12a.begin(BaudRate, DirectionPin, &Serial);
 ax12a.setEndless(ID1, OFF);
 //master

}

void loop()
{
    while (Serial.available() == 0){}//Doesn't move on until new inputs detected

    while (Serial.available() > 0){  // Flush old input characters
      curr_servo_ang = Serial.parseInt();
      Serial.println("Received upper servo angle: "+ String(curr_servo_ang));
        
      if(prev_servo_ang != curr_servo_ang)
      {
        move_to_angle(curr_servo_ang, 100);
        prev_servo_ang = curr_servo_ang;
      }
    }
    while(Serial.available() > 0){Serial.read();}
}

void move_to_angle(int ang, int sp)
{
  
  ax12a.begin(BaudRate, DirectionPin, &Serial1);
  ax12a.begin(BaudRate, DirectionPin, &Serial2);
  
  ax12a.moveSpeed(ID1, int((ang*3.41333)+512), sp);
  ax12a.end();
}
