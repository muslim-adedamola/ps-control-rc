#include <Wire.h> // Import (include) the Wire library
#include <MPU6050_tockn.h> // Import (include) the MPU6050_tockn library (install it if it is not already installed)
#include <NewPing.h>

const int ENA = 11;
const int ENB = 10;
const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 7;
const int IN4 = 6;
const int TRIG_PIN = 4;
const int ECHO_PIN = 5;

MPU6050 mpu6050(Wire); // Create an instance of MPU6050 and give it a custom name (mpu6050 in this case). Pass Wire as a parameter to this instance
const int DO = 0; /* Specify the pin number for the digital output (DO) pin of the FC-03. This pin SHOULD be connected to an interrupt
pin. The arduino uno has two (2) interrupt pins; Interrupt Pin 0 (Pin 2) and Interrupt Pin 1 (Pin 3). We choose to use Interrupt Pin 0 */

NewPing sonar (TRIG_PIN, ECHO_PIN, 200);

const int numberOfSlots = 20; // Our wheel encoder disk has twenty (20) slots

const float WHEEL_DIAMETER = 6.60; // Wheel diamter is 6.60cm
const float distanceCalibration = 1.248;

const float distancePerTick = (PI * WHEEL_DIAMETER/numberOfSlots) * distanceCalibration; // Pretty obvious...duh

float angle_offset = 0;

const float obstacleDist = 10; // Obstacles should not be 10 cm of nearer to the robot

volatile float distanceTravelled = 0; /* Variable that stores the distance travelled. This variable is "volatile" because its 
value can change spontaneously (even in the middle of running a line of code) */

volatile float numberOfTicksSinceLastSpeedEstimate = 0;

volatile boolean isTurning = false;

int SPEED = 65; // We will be driving at 65 (out of 255) initially

const float totalDistance = 300; 

const unsigned long travelTimeConstraint = 2000;

unsigned long travelTime;
unsigned long travelTimeAtLastSpeedEstimate;

int numberOfTurns = 0;
unsigned long startTime;

void setup() 
{
  // put your setup code here, to run once:

  // Define Pin Modes
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);

  pinMode (2, INPUT_PULLUP);

  /* Attach the DO pin to the interrupt pin (Pin 2 (0) in this case). 
   *  The attachInterrupt() function takes three(3) parameters:
   *  The interrupt pin number (Pin 2(0) in this case),
   *  The function to be called when the encoder "ticks" (in this case the pulse() function is called), and
   *  A mode (in this case the mode is RISING. This specifies that interrupt should be triggered when 
   *  the pin value goes from LOW to HIGH. Other flags exist like: CHANGE 
   *  (triggers interrupt when the pin goes from low to high or high to low), 
   *  FALLING (triggers interrupt when the pin goes from high to low), etc
   */
  attachInterrupt(DO, pulse, RISING); 
  
  Serial.begin (9600); 

  // Uncomment the next two lines if you have a functioning bluetooth sensor 
  // while (!Serial.available());
  // Serial.println("Connected!");
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

  Serial.println(angle_offset);
  delay(5000);
  Serial.println("Starting...");
  startTime = millis();
  travelTime = millis();
  travelTimeAtLastSpeedEstimate = millis();
}

void loop() 
{
  // put your main code here, to run repeatedly:

  while (obstacle())
  {
    stopNow();
  }
  
  if (distanceTravelled >= totalDistance) // After travelling 150cm (1.5m) in one direction
  {
    stopNow();
    Serial.print ("Robot moved ");
    Serial.print (distanceTravelled);
    Serial.println ("cm"); // Print your distance to the serial monitor
    Serial.print("Travel time: ");
    Serial.print((millis() - startTime));
    Serial.println("milliseconds");
    Serial.print("Average Speed (cm/ms): ");
    Serial.println(distanceTravelled / (millis() - startTime));
    while (true); // Freeze the program at this point
  }

  else
  {
    driveStraight(0);
    adjustSpeed();
  }
}

/*
 * Function to check if an obstacle is too close to the robot
 */
boolean obstacle()
{
  return sonar.ping_cm() <= obstacleDist; 
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

/*
 * Function to stop the robot
 */
void stopNow()
{
  analogWrite(ENA, 0);  
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW);    
}

/*
 * Function to adjust robot speed in order to keep to time constraints
 * Logic is simple:
 * Calculate the average current speed (in cm/ms) by taking the ratio of the distanceTravelledSinceLastSpeedEstimate to the
 * travel time difference from the last speed estimate
 * Calculate the speed needed to complete the rest of the journey in time (desired_speed) by taking the
 * ratio of the remaining distance to the remaining time.
 * Calculate the PWM value of the desired speed and change the current PWM value to this new value
 * This approached worked very well, probably better than the old approach but I cannot say with certainty
 * as I didn't test "extensively" 
 */
void adjustSpeed()
{
  /* 
   * Do not adjust speed if the distance travelled from last time time (number of 
   * ticks since then) is too small (less than 2)
   */
  if (numberOfTicksSinceLastSpeedEstimate < 2) 
  {
    return;
  }
  
  float distance = totalDistance - distanceTravelled; 

  /* 
   *  If the time left is negative (i.e, the alloted time has elapsed) just move at "maximum" speed
   */
  if (travelTimeConstraint < (millis() - travelTime)) 
  {
    SPEED = 180;
    return;
  }
  unsigned long timeLeft = travelTimeConstraint - (millis() - travelTime);
  
  float desired_speed = distance / timeLeft;

  float distanceTravelledSinceLastSpeedEstimate = numberOfTicksSinceLastSpeedEstimate * distancePerTick;
  
  float actual_speed = distanceTravelledSinceLastSpeedEstimate / (millis() - travelTimeAtLastSpeedEstimate);

  float newSpeed = (float(SPEED) * desired_speed) / actual_speed;
  
  SPEED = int(constrain(round(newSpeed), 65, 180));

  travelTimeAtLastSpeedEstimate = millis();
  numberOfTicksSinceLastSpeedEstimate = 0;
}

/*
 * Function called each time the encoder "ticks"
 */
void pulse()
{
  if (isTurning)
  {
    return;
  }
  distanceTravelled += distancePerTick;
  numberOfTicksSinceLastSpeedEstimate++; 
}
