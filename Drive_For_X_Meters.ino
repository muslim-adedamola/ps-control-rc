#include <Wire.h> // Import (include) the Wire library
#include <MPU6050_tockn.h> // Import (include) the MPU6050_tockn library (install it if it is not already installed)

MPU6050 mpu6050(Wire); // Create an instance of MPU6050 and give it a custom name (mpu6050 in this case). Pass Wire as a parameter to this instance
const int DO = 0; /* Specify the pin number for the digital output (DO) pin of the FC-03. This pin SHOULD be connected to an interrupt
pin. The arduino uno has two (2) interrupt pins; Interrupt Pin 0 (Pin 2) and Interrupt Pin 1 (Pin 3). We choose to use Interrupt Pin 0 */

const int ENA = 11;
const int ENB = 10;
const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 7;
const int IN4 = 6;

const int numberOfSlots = 20; // Our wheel encoder disk has twenty (20) slots

const float WHEEL_DIAMETER = 6.60; // Wheel diamter is 6.60cm
const float distanceCalibration = 1.248;

const float distancePerTick = (PI * WHEEL_DIAMETER/numberOfSlots) * distanceCalibration; // Pretty obvious...duh

float angle_offset = 0;

volatile float distanceTravelled = 0; /* Variable that stores the distance travelled. This variable is "volatile" because its 
value can change spontaneously (even in the middle of running a line of code) */

volatile boolean isTurning = false;

const int SPEED = 70; // We will be driving at 70 (out of 255)

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
}

void loop() 
{
  // put your main code here, to run repeatedly:
  
  if (distanceTravelled >= 100) // After travelling 100cm (1m) in one direction
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
  }
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
 * Function to drive forward
 */
void goForward()
{
  analogWrite(ENA, SPEED);  
  analogWrite(ENB, SPEED);

  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW); 
}

/*
 * Function to turn right
 */
void turnRight()
{
  analogWrite(ENA, SPEED);  
  analogWrite(ENB, SPEED);

  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);  
}

/*
 * Function to turn left
 */
void turnLeft()
{
  analogWrite(ENA, SPEED);  
  analogWrite(ENB, SPEED);

  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);  
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
 * Function called each time the encoder "ticks"
 */
void pulse()
{
  if (isTurning)
  {
    return;
  }
  distanceTravelled += distancePerTick; 
}
