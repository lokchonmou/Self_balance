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
long cumulative_acc;
int init_acc, init_gyro, acc_value, gyro_value;

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
  init_gyro = 75;
  sample_time = 200;
  
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
  cumulative_acc += acc_value;
  output = - (Kp * acc_value + Kd * gyro_value  + Ki * cumulative_acc);
  output = constrain(output, -255, 255);
  }

  if (output > 0)
  {
    analogWrite(E1, output);
    digitalWrite(M1, HIGH);
    analogWrite(E2, output);
    digitalWrite(M2, HIGH);
  }
  else{
    analogWrite(E1, -output);
    digitalWrite(M1, LOW);
    analogWrite(E2, -output);
    digitalWrite(M2, LOW);
  }
  
  Kp = map(analogRead(A0), 0, 1023, 0, 15);
  Ki = map(analogRead(A1), 0, 1023, 0, 10);
  Kd = map(analogRead(A2), 0, 1023, 0, 10);
}
