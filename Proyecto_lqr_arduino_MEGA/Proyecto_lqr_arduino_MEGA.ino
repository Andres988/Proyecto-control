 #include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

//#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

///////////////////////////////  VARIABLES ENCODER   ///////////////
const int channelPinA = 18;
const int channelPinB = 19;
 
const int timeThreshold = 5;
long timeCounter = 0;
 
//const int maxSteps = 255;
volatile int ISRCounter = 0;
int counter = 0;
double pos_x=0;
double pos_old=0;
double pos_old_1=0;
bool IsCW = true;

///////////////////////////////////////////////////////////////////

///////////////////////////////Variables de robot///////////////////
int pin_output1 = 4;
int pin_output2 = 5;
int pin1 = 48;
int pin2 = 46;
int pin3 = 52;
int pin4 = 53;
int pin_en = 50;

//Define Variables we'll be connecting to
double Setpoint, Input_mpu=0, Output;
double ang_k=0;
double ang_old_k=0;
double Input_mpu_k=0;
//Specify the links and initial tuning parameters
double x[4];
double u =0;
double k_lqr[4]={31.6228, 115.5214, -482.0973, -103.4864};  
double PWM=0;
double v_motor = A0;
double ang_old=0;

/////////////////////////////////////////////////////////////////



const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication


/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() 
{
  ///////////// PINES MOTOR  /////////////
  pinMode(pin_output1, OUTPUT);
  pinMode(pin_output2, OUTPUT);
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
  pinMode(pin_en, OUTPUT);
  
  ////////////////////////////////////////

  ////////////////////////// PINES ENCODER E INTERRUOCION //////////

   pinMode(channelPinA, INPUT_PULLUP);
   pinMode(channelPinB, INPUT_PULLUP);

   attachInterrupt(digitalPinToInterrupt(channelPinA), doEncodeA, CHANGE);
   attachInterrupt(digitalPinToInterrupt(channelPinB), doEncodeB, CHANGE);
  
  //////////////////////////////////////////////////////////


  
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
  Setpoint = 2;//92.7
  digitalWrite(pin_en,HIGH);
}

void loop() {
  /* Update all the values */
  
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  float dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  //gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  //gyroYangle += gyroYrate * dt;
  gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif


#if 0 // Set to 1 to print the temperature
  Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
#endif
  
  //delay(2);
/////////////////////////    CODIGO DE CONTROL      //////////////////////////
   
   ////////////////////// LECTURA DE DATOS  ////////////////////
   Input_mpu =compAngleX;// kalAngleX;  // compAngleY    ANGULO
   //Input_mpu_k=kalAngleX;
   if (counter != ISRCounter)
    {
       counter = ISRCounter;
       
       pos_x = counter*0.00078;
    }// POSICION POR PULSOS
   //delay(100);

   /////////////////////////////////////////////////////////////////////


   ///////////////////////// DEFINICION DE ESTADOS  //////////////////////
   //if(dt!)
   x[0]=pos_x;                                                      //Posicion lineal m
   x[1]=(x[0]-pos_old_1)/dt;//(analogRead(v_motor)*0.0033)/1023;   //Velocidad lineal m/s
   x[2]=(Setpoint*(3.1416/180))-(Input_mpu*(3.1416/180)); 
   //ang_k=(Setpoint*(3.1416/180))-(Input_mpu_k*(3.1416/180));//Angulo rad
   x[3]=(x[2]-ang_old)/dt;                                       //Velocidad angular rad/s                                            
  
   u = x[0]*k_lqr[0] + x[1]*k_lqr[1] + x[2]*k_lqr[2] + x[3]*k_lqr[3];
  // u = x[2]*k_lqr[2] + x[3]*k_lqr[3];
   pos_old_1=pos_old;
   pos_old=x[0];
   //ang_old_k=ang_k;
   ang_old=x[2];
  if(u>0)
  {
    digitalWrite(pin1,HIGH);
    digitalWrite(pin2,LOW);
    digitalWrite(pin3,HIGH);
    digitalWrite(pin4,LOW);
  }
  else
  {
    //Input_mpu = Input_mpu*(-1);
    digitalWrite(pin1,LOW);
    digitalWrite(pin2,HIGH);
    digitalWrite(pin3,LOW);
    digitalWrite(pin4,HIGH);
  }


  PWM=(abs(u));
  if(PWM>255)
  {
    PWM=255;
  }
  if((Input_mpu>Setpoint+45 || Input_mpu<Setpoint-45))// || (Input_mpu >Setpoint +0.3 && Input_mpu <Setpoint -0.3))
  {
    analogWrite(pin_output1, 0);
    analogWrite(pin_output2, 0);
    //digitalWrite(pin_en,LOW);
  }
  else
  {
    //digitalWrite(pin_en,HIGH);
    analogWrite(pin_output1, PWM);
    analogWrite(pin_output2, PWM);
  }

  
/////////////////////////////////////////////////////////////////////////////////////

  ///////////////////// IMPRIMIR DATOS /////////////////////////////////

  Serial.print("\t");
  Serial.print("Angulo: "); Serial.print(Input_mpu); Serial.print("\t");
  Serial.print("Posicion:"); Serial.print(pos_x); Serial.print("\t");
  Serial.print("vel_ang:"); Serial.print(x[3]); Serial.print("\t");
  Serial.print("Vel_lin:"); Serial.print(x[1]); Serial.print("\t");
  //Serial.print("Motor:"); Serial.print(PWM); Serial.print("\t");
  
  Serial.print("\r\n");
  delay(25);
  //////////////////////////////////////////////////////////////////////
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
void doEncodeA()
{
   if (millis() > timeCounter + timeThreshold)
   {
      if (digitalRead(channelPinA) == digitalRead(channelPinB))
      {
         IsCW = true;
         //if (ISRCounter + 1 <= maxSteps) 
         ISRCounter++;
      }
      else
      {
         IsCW = false;
         //if (ISRCounter - 1 > 0) 
         ISRCounter--;
      }
      timeCounter = millis();
   }
}
 
void doEncodeB()
{
   if (millis() > timeCounter + timeThreshold)
   {
      if (digitalRead(channelPinA) != digitalRead(channelPinB))
      {
         IsCW = true;
         //if (ISRCounter + 1 <= maxSteps) 
         ISRCounter++;
      }
      else
      {
         IsCW = false;
         //if (ISRCounter - 1 > 0) 
         ISRCounter--;
      }
      timeCounter = millis();
   }
}
