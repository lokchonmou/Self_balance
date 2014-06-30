#include <PID_v1.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Wire.h>

int rawSixDof[6];

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

double Kp, Ki, Kd, input, output;
unsigned long timer, sample_time;
long cumulative_angle;
float init_acc, init_gyro, acc_value, gyro_value, angle, gyro_angle;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd, DIRECT);

byte E1 = 5;
byte M1 = 4;
byte E2 = 6;
byte M2 = 7;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
  
  init_acc=0;
  init_gyro = -6;
//  sample_time = 200;
  Setpoint = 0;  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop(){
 
  sixDOF.getRawValues(rawSixDof);
  
  acc_value = rawSixDof[1]- init_acc;
  gyro_value = rawSixDof[3] - init_gyro;
  
  if (millis() - timer >= sample_time)   { //do the following every 200ms
  gyro_angle += gyro_value * sample_time / 1000;
  gyro_angle = constrain(gyro_angle, -15, 15);
  angle = 0.8 * acc_value + 0.2 * gyro_angle;
  }
  
  Input = angle;
  myPID.Compute();

  if (Output > 0)
  {
    analogWrite(E1, Output);
    digitalWrite(M1, HIGH);
    analogWrite(E2, Output);
    digitalWrite(M2, HIGH);
  }
  else{
    analogWrite(E1, -Output);
    digitalWrite(M1, LOW);
    analogWrite(E2, -Output);
    digitalWrite(M2, LOW);
  }
  
  Kp = map(analogRead(A0), 0, 1023, 0, 10);
  Ki = map(analogRead(A1), 0, 1023, 0, 3);
  Kd = map(analogRead(A2), 0, 1023, 0, 3);
  myPID.SetTunings(Kp, Ki, Kd);
}
