#include <Streaming.h> //to print stuff easier
#include <AX12A.h>


#define DirectionPin (10u)
#define ID            (1u)
unsigned char ID1=1;
#define BaudRate   (1000000)

//This can be edited
float yawstep=15;
float pitchstep=15;
int timestep=1;

float max_pitch_angle=90;
float min_pitch_angle=-90;

float min_yaw_angle=0;
float max_yaw_angle=300;
    
float init_lower_servo_ang=0; //initial values that can be changed
float init_upper_servo_ang=0;

void setup()
{
 Serial.begin(1000000);
 

 pinMode(13, OUTPUT);
 Serial.println("Change the servo orientation by following directions") ; //360?

 //-150-150 for lower servo
 //-90-90 for upper servo, 0 degrees is upright
 ax12a.begin(BaudRate, DirectionPin, &Serial);



 ax12a.setEndless(ID1, OFF);
 //master
 
}

void loop()
{

    Serial.println("Enter lower servo orientation in degrees as a float(-150°to 150°)."); 
    while (Serial.available()) Serial.read();  // Flush old input characters
    while (Serial.available()==0) { }
    float lower_servo_ang= Serial.parseFloat();
    
    Serial.println("Enter upper servo orientation in degrees as a float (-90°to 90°).") ;//>180
    while (Serial.available()) Serial.read();  // Flush old input characters
    while (Serial.available()==0) { }
    float upper_servo_ang= Serial.parseFloat();
    Serial << "(Lower Servo Orientation, Upper Servo Orientation): (" << lower_servo_ang << ", " << upper_servo_ang << ")\n";    
    custom_sequence(timestep, lower_servo_ang,  upper_servo_ang);
    //testing_sequence(timestep); //use this if want many different orientations
    
}

void move_to_angleself(int num, float ang, int sp, int t)
{
  if(num==2){ax12a.begin(BaudRate, DirectionPin, &Serial2);}
  if(num==1){ax12a.begin(BaudRate, DirectionPin, &Serial1);}
  ax12a.moveSpeed(ID1, int((ang*3.41333)+512), sp);
  delay(t);
  ax12a.end();
}
void testing_sequence(int sec){
  move_to_angleself(2, -90, 200, 1000*sec+2000); //initialize to base position
  for (int j=1; j<=(max_pitch_angle-min_pitch_angle)/pitchstep; j++){
    move_to_angleself(1, -150, 250, 1000*sec+3000);
    for(int i=1; i<=(max_yaw_angle-min_yaw_angle)/yawstep; i++)
    {
      move_to_angleself(1, -150+yawstep*i, 100, 1000*sec);
    }
    move_to_angleself(2, -90+pitchstep*j, 80, 1000*sec);
  }
    delay(sec*1000); 
}
void custom_sequence(int sec, float ang1, float ang2){
  //lower is 1
  //upper is 2
  move_to_angleself(2, ang2, 80, 1000*sec); 
  delay(1000);

  move_to_angleself(1, init_lower_servo_ang+ang1, 100, 1000*sec);
  delay(1000);
}

void move_to_anglesend(float ang, int sp, int t)
{
  ax12a.moveSpeed(ID1, (ang*309/90)+512, sp);
  delay(t);
}
void move_to_anglereceive(float ang, int sp, int t)
{
  ax12a.moveSpeed(ID1, (ang*309/90)+512, sp);
  delay(t);
}
