#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <PID_v1.h>
#include <Wire.h>

int angles[6]; // yaw pitch roll
double result[2];
double Setpoint[2], Input[2], Output[3];

//----------------------------------------------------
double Ka = 0;
double Kb = 1;

PID myPID(&Input[0], &Output[0], &Setpoint[0],25,0,1, DIRECT);
PID myPID2(&Input[1], &Output[1], &Setpoint[1],0.02, 0.0005, 0.001, REVERSE);
//------------------------------------------------

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
byte E1 = 5;
byte M1 = 4;


void setup() 
{ 
  Setpoint[0] = -10.50;
  Setpoint[1] = 0;

  Serial.begin(9600);
  Wire.begin();
  for (int i = 4; i<8; i++)
  {
    pinMode(i, OUTPUT);
  }
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);

 myPID.SetOutputLimits(-255, 255);
  myPID2.SetOutputLimits(-255, 255);
  myPID.SetMode(MANUAL);
  myPID2.SetMode(MANUAL);

}

void loop() { 
  if (millis() >10000  && millis() < 10200) 
  {
    myPID.SetMode(AUTOMATIC);
    myPID2.SetMode(AUTOMATIC);
  }
  // sixDOF.getEuler(angles);

  // Serial.print(angles[0]);
  //Serial.print(" | ");  
  // Serial.print(angles[1]);
  // Serial.print(" | ");
  // Serial.println(angles[2]);



  // Serial.println(Output); 
  sixDOF.getRawValues(angles); 
  result[0] = atan((double)angles[2]/angles[1])*180/3.1415926535897932384626433832795;
  result[1] = (double)angles[3]*180/3.1415926535897932384626433832795;
  Input[0] = result[0]; 
  Input[1] = result[1];
  myPID.Compute();
  myPID2.Compute();

   Output[2] = Ka*Output[0] + Kb*Output[1];
   if (Output[2] >255) Output[2] = 255;
   else if (Output[2] <-255) Output[2] = -255;

  Serial.print(result[0]);
  Serial.print(" | ");
  Serial.print(result[1]);
  Serial.print(" | ");
  Serial.println(Output[2]);

/*  if (Output[2]<0)
   {
   digitalWrite(M1, HIGH);
   
   analogWrite(E1, abs(Output[2]));
   
   }
   else
   {
   digitalWrite(M1, LOW);
   
   analogWrite(E1, abs(Output[2]));
   
   } */
}









