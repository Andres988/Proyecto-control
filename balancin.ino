

/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/
#include <MPU6050_tockn.h>
#include <Wire.h>

#include <PID_v1.h>

#define PIN_OUTPUT1 5
#define PIN_OUTPUT2 11
#define PIN1 7
#define PIN2 6
#define PIN3 9
#define PIN4 10
int pin_en = 8;
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=20, Ki=3, Kd=0.35; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
MPU6050 mpu6050(Wire); //ubicacion de mpu
void setup()
{
  Serial.begin(9600);  //inicializando mpu
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  //initialize the variables we're linked to
  Input = mpu6050.getAngleY();
  Setpoint = 92.7;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  digitalWrite(pin_en,HIGH);
  
}
void loop()
{
  Ki=3;
  mpu6050.update();
  Input = mpu6050.getAngleY();
  if(Input>0)
  {
    Input = Input*(-1);
    digitalWrite(PIN1,HIGH);
    digitalWrite(PIN2,LOW);
    digitalWrite(PIN3,HIGH);
    digitalWrite(PIN4,LOW);
  }
  else
  {
    
    digitalWrite(PIN1,LOW);
    digitalWrite(PIN2,HIGH);
    digitalWrite(PIN3,LOW);
    digitalWrite(PIN4,HIGH);
  }
  if (Input==Setpoint)
  {
    Ki=0;
  }
  
  myPID.Compute();
  if(Input>45 || Input<135) //|| (Input < 0.2 && Input > -0.2))
  {
    analogWrite(PIN_OUTPUT1, 0);
    analogWrite(PIN_OUTPUT2, 0);
  }
  else
  {
    analogWrite(PIN_OUTPUT1, Output);
    analogWrite(PIN_OUTPUT2, Output);
  }
  
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\tsetpoint : ");
  Serial.print(Setpoint);
  Serial.print("\toutput : ");
  Serial.println(Output);
 
}
