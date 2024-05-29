#include <Streaming.h> //to print stuff easier
#include <AX12A.h>

#define DirectionPin (10u)
#define ID            (1u)
unsigned char ID1 = 1;
#define BaudRate   (1000000)

//This can be edited
    
int curr_servo_ang=0;
int prev_servo_ang=0;

int curr_el_ang = 0;
int prev_el_ang = 0;
int curr_az_ang = 0;
int prev_az_ang = 0;

void setup()
{
 Serial.begin(1000000);

 pinMode(13, OUTPUT);

 //-90-90 for upper servo, 0 degrees is upright
 ax12a.begin(BaudRate, DirectionPin, &Serial);
 ax12a.setEndless(ID1, OFF);
 //master

}
int az_el_string_to_array(String curr_az_el)
{
  static int result[2];
  int comma_index = curr_az_el.indexOf(',');
  int end_paren_index = curr_az_el.indexOf(')');
  int az = curr_az_el.substring(1,comma_index).toInt();
  int el = curr_az_el.substring(comma_index+1, end_paren_index).toInt();


  result[0] = az;
  result[1] = el;
  
  return result;
}
void loop()
{
    while (Serial.available() == 0){}//Doesn't move on until new inputs detected

    while (Serial.available() > 0){  // Flush old input characters
      String curr_az_el = Serial.readString();     
      Serial.println("Received az el angle: "+ String(curr_az_el));
      int* result = az_el_string_to_array(curr_az_el);
      int curr_az_ang = result[0];
      int curr_el_ang = result[1];
      if (curr_az_ang != prev_az_ang)
      {
        move_to_angle(1, curr_az_ang, 100);
        prev_az_ang = curr_az_ang;
      }
      if (curr_el_ang != prev_el_ang)
      {
        move_to_angle(2, curr_el_ang, 100);
        prev_el_ang = curr_el_ang;
      }
      
      //if(prev_servo_ang != curr_servo_ang)
      //{
        //move_to_angle(curr_servo_ang, 100);
        //prev_servo_ang = curr_servo_ang;
      //}
    
    }
    while(Serial.available() > 0){Serial.read();}
}
void move_to_angle(int servo, int ang, int sp)
{
  //lower is 1
  //upper is 2
  if(servo==2){ax12a.begin(BaudRate, DirectionPin, &Serial2);}
  if(servo==1){ax12a.begin(BaudRate, DirectionPin, &Serial1);}
  //ax12a.begin(BaudRate, DirectionPin, &Serial1);
  //ax12a.begin(BaudRate, DirectionPin, &Serial2);
  
  ax12a.moveSpeed(ID1, int((ang*3.41333)+512), sp);
  ax12a.end();
}
