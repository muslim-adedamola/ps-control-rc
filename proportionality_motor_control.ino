int ENA=5;
int ENB=3;
int IN1=8;
int IN2=9;
int IN3=6;
int IN4=7;

int echo=2;
int trig=11;

float k=0.1;   //gain of the controller

int SPEED;
float distance;

float ref_distance=30.0;
float error;

float distarray[]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; //variable to store 10 successive distance  in an array


void setup() {
  Serial.begin(9600);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);
}

 
void loop() {
  Serial.println(getDistance());
 for(int i=9;i>0;i--){
  distarray[i]=distarray[i-1]; //store distance reading in an array
  }
  distarray[0]=getDistance(); 

for(int i=0;i<10;i++){
  distance = distance + distarray[i]; //sum of all the distance reading
  }
  distance=distance/10;   //average distance

  error=distance-ref_distance; //error: difference between the distance and reference distance
  
  if(error<0){        //if error is a negative
    error=error*(-1); //set the value of error to a positve value
    error=error*k;    // multiply error by the gain.
    SPEED=map(error,0,30,76.5,255);  //map error to the pwm value 
    movebackward();
    }
    else if(error>0){ //if error is positive
      error=error*k;  //multiply the value of error by the gain
      SPEED=map(error,0,30,70,255);   //map the value of error to the pwm value
      moveforward();
      }
      else{
        stopmoving();
        }
  
   
}


void moveforward(){
  analogWrite(ENA, SPEED);  
  analogWrite(ENB, SPEED);

  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH); 
  }
  
void movebackward(){
  analogWrite(ENA, SPEED);  
  analogWrite(ENB, SPEED);

  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);   
  }
void stopmoving(){
  analogWrite(ENA, 0);  
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW);
  }  
  
 float getDistance(){
  float duration;
  float distance;
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  duration=pulseIn(echo,HIGH);
  distance=duration*0.017;
  return distance;
  
  }
