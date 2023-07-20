#include <MPU6050_tockn.h>
#include <Wire.h>

#define IN4 6
#define IN3 7
#define IN2 8
#define IN1 9
#define ENB 10
#define ENA 11

MPU6050 mpu6050(Wire); // Create an instance of MPU6050 and give it a custom name (mpu6050 in this case). Pass Wire as a parameter to this instance

float angle_offset = 0;

float desiredAngle = 45;

int SPEED = 60; 

void setup() 
{
  // put your setup code here, to run once:
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);

  Serial.begin (9600); 
  
  Wire.begin();
  mpu6050.begin();
  
  mpu6050.calcGyroOffsets(true); /* This calculates the offset values of the gyroscope. When this function is called with (true) as 
  is the case here, you can see state of calculating calibration in the serial monitor */
  

  /* We want to calculate the angular offset of our robot so we get the average of
   *  100 "Z" angle readings
   */
  for (int i = 0; i < 100;i++)
  {
    mpu6050.update();
    angle_offset += mpu6050.getAngleZ();
  }
  
  angle_offset = angle_offset/100.00;  // Calculating the average as explained above
}

void loop() 
{
  // put your main code here, to run repeatedly:
  driveStraight(desiredAngle);
}


/*
 * Function to get angular position from the IMU
 */
float getAngle()
{
  mpu6050.update(); // You must update() before getting any data from the MPU-6050
  
  float theta = mpu6050.getAngleZ()- angle_offset; // Subtract the angle_offset (error) from our angular measurement

  return theta;
}

/*
 * Function to drive straight at a desired angle
 */
void driveStraight(float desired_angle)
{
  float error = getAngle() - desired_angle;
  float k = 1;
  float action = k * error;

  int leftSpeed = constrain(SPEED - int(round(action)), -200, 200);
  int rightSpeed = constrain(SPEED + int(round(action)), -200, 200);

  analogWrite(ENA, abs(rightSpeed));
  analogWrite(ENB, abs(leftSpeed));

  if (rightSpeed < 0)
  {
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH);   
  }
  else
  {
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW); 
  }

  if (leftSpeed < 0)
  {
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH);   
  }
  else
  {
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW); 
  }
  
}
