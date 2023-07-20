#include <Wire.h> // Import (include) the Wire library
#include <MPU6050_tockn.h> // Import (include) the MPU6050_tockn library (install it if it is not already installed)

MPU6050 mpu6050(Wire); // Create an instance of MPU6050 and give it a custom name (mpu6050 in this case). Pass Wire as a parameter to this instance
const int DO = 2; /* Specify the pin number for the digital output (DO) pin of the FC-03. This pin SHOULD be connected to an interrupt
pin. The arduino uno has two (2) interrupt pins; Interrupt Pin 0 (Pin 2) and Interrupt Pin 1 (Pin 3). We choose to use Interrupt Pin 0 */

const int ENA = 9;
const int ENB = 10;
const int IN1 = 3;
const int IN2 = 4;
const int IN3 = 5;
const int IN4 = 6;
const int SPEED = 255; // We will be driving at 150 (out of 255)

float angle_offset = 0;
float Ux = 0;
float Uy = 0;
float X = 0;
float Y = 0;
float theta = 0.00;
float Ax = 0;
float Ay = 0;
const long interval = 3000;
unsigned long previousMillis = 0;

void pulse();
void goForward();
int updatepulse(int dt);

void setup()
{
  Serial.begin(9600);
  while(!Serial.available())  //waiting for receiver to send something
  {
    }
  Serial.println("Starting...");
  delay(5000);

  // Define Pin Modes
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);

  pinMode (DO, INPUT_PULLUP);

 attachInterrupt(DO, pulse, CHANGE); 
  
  Wire.begin();
  mpu6050.begin();
  
  mpu6050.calcGyroOffsets(true); /* This calculates the offset values of the gyroscope. When this function is called with (true) as 
  is the case here, you can see state of calculating calibration in the serial monitor */
  

/* We want to calculate the angular offset of our robot so we get the average of
 *  100 "Z" angle readings
 */
for (int i = 0; i < 100; i++)
{
  mpu6050.update();
  angle_offset += mpu6050.getAngleZ();
}

  angle_offset = angle_offset/100.00;  // Calculating the average as explained above

  Serial.println(angle_offset);
  delay(10000);
}

void loop(){
  Serial.println("Working...");
  delay(500);

  goForward();
  pulse(); 
}

void goForward()
{
  analogWrite(ENA, SPEED);  
  analogWrite(ENB, SPEED);

  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW); 
}

int updatepulse(int dt){
     int divisor = mpu6050.getAngleZ() - angle_offset;
     int theta_initial = divisor % 360;
     theta = float(theta_initial);
     
     Ax = mpu6050.getAccX();
     Ay = mpu6050.getAccY();

     Ax = 0.1 * Ax;
     Ay = 0.1 * Ay;

     float Vx = Ux + (Ax*dt);
     float Vy = Uy + (Ay*dt);

     X = X + ( (Ux * dt) + (0.5 * Ax *(dt^2)));
     Y = Y + ( (Uy * dt) + (0.5 * Ay * (dt^2)));

    Ux = Vx;
    Uy = Vy;

    Serial.print("X-axis: "); Serial.println(X); Serial.print("Y-axis: "); Serial.println(Y); Serial.print("Theta: "); Serial.println(theta);
  }


void pulse() {
    unsigned long currentMillis = millis();

  if(currentMillis - previousMillis >= interval) {
     int da = currentMillis - previousMillis;
     updatepulse(da);
     previousMillis = currentMillis;
  }
}


