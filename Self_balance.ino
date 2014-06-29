#include <PID_v1.h>

float yaw, pitch, roll;

double Setpoint, Input, Output;
double Kp, Ki, Kd;
PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd, DIRECT);
byte E1 = 5;
byte M1 = 4;
byte E2 = 6;
byte M2 = 7;

void setup()
{
  Serial.begin(57600);
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
  if (Serial.available())
  {
    yaw = Serial.parseFloat();
    pitch = Serial.parseFloat();
    roll = Serial.parseFloat();
  }

  //  Serial.print(yaw);
  //  Serial.print(", ");
  //  Serial.print(pitch);
  //  Serial.print(", ");
  //  Serial.println(roll);
  
  Input = pitch;
  myPID.Compute();
  //Serial.println(Output);  
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
  Kp = map(analogRead(A0), 0, 1023, 0, 15);
  Ki = map(analogRead(A1), 0, 1023, 0, 5);
  Kd = map(analogRead(A2), 0, 1023, 0, 5);
  myPID.SetTunings(Kp, Ki, Kd);
}






