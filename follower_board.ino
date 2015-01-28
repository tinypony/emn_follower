#include <stdlib.h>
#include <Servo.h>
#define INTEG_SIZE 32

const byte analogPin2 = 40;
const byte analogPin3 = 41;
const byte analogPin4 = 42;
const byte analogPin5 = 43;
const byte analogPin6 = 44;
const byte analogPin7 = 45;

int calibration[6];
int lut[6] = {-30, -20, -10, 10, 20, 30};

int integrationSize = INTEG_SIZE;
int integrationArray[INTEG_SIZE];
int integrationPtr = 0;

//Tune this!
int P=2;
float I=0.2;
float D=0.15;

int outMax = 750;
int outMin = -750;
int lastInput = 0;
double setpoint = 0;     // Initial setpoint is 0
int scaleFactor = 10;

int threshold = 50;
Servo servo;
const int servoPin = 26;
int servoCenter = 93;


void setup()
{
  Serial.begin(9600);
  pinMode(analogPin2, INPUT);
  pinMode(analogPin3, INPUT);
  pinMode(analogPin4, INPUT);
  pinMode(analogPin5, INPUT);
  pinMode(analogPin6, INPUT);
  pinMode(analogPin7, INPUT);  
  calibrate();
  int n = 0;
  for(int i =0; i< INTEG_SIZE; i++)
  {
    integrationArray[i] = 0; 
  }
  servo.attach(servoPin);
}

void calibrate() {
   int sensor[6];
   
   for(int i=0; i < 10; i++ ) {
     readSensor(sensor);
     
     if(i == 0) {
       memcpy(calibration, sensor, 6 * sizeof(int)); 
     } else {
       for(int j=0; j<6; j++) {
         calibration[j] = (sensor[j] + i * calibration[j]) / (i+1); 
         if(j == 0) {
           Serial.println(calibration[j]);
         }
       } 
     }
   }
   
   outputValues(calibration, 1);
}

int getArrayValue(int* sensor) {
  int total = 0;
  int n = 0;
  
   for(int i=0; i<6; i++) {
     if(sensor[i] > threshold) {
        total += lut[i];
        n++;
     }
   } 
   
   if(n > 0) {
     return total/n;
   } else {
     return 0; 
   }
}

void readSensor(int* array) {
  array[0] = analogRead(analogPin2);
  array[1] = analogRead(analogPin3);
  array[2] = analogRead(analogPin4);
  array[3] = analogRead(analogPin5);
  array[4] = analogRead(analogPin6);
  array[5] = analogRead(analogPin7);
}

void outputValues(int* sensor, int newline)
{
  Serial.print("0:");
  Serial.print(sensor[0]);
  Serial.print("\t");
  Serial.print("1:");
  Serial.print(sensor[1]);
  Serial.print("\t");
  Serial.print("2:");
  Serial.print(sensor[2]);
  Serial.print("\t");
  Serial.print("3:");
  Serial.print(sensor[3]);
  Serial.print("\t");
  Serial.print("4:");
  Serial.print(sensor[4]);
  Serial.print("\t");
  Serial.print("5:");
  Serial.println(sensor[5]);
}

void standardize(int* sensor) {
 sensor[0] -= calibration[0];
 sensor[1] -= calibration[1];
 sensor[2] -= calibration[2];
 sensor[3] -= calibration[3];
 sensor[4] -= calibration[4]; 
 sensor[5] -= calibration[5];
}

void loop()
{
  int sensor[] = {0, 0, 0, 0, 0, 0};
  
  readSensor(sensor);
  standardize(sensor);
  int value = getArrayValue(sensor);
  follow(getPID(value));
//  outputValues(sensor, 1);
//  outputLightPosition(value);
}

long integrate() {
  long retval = 0;
  for(int i=0; i<INTEG_SIZE; i++) {
    retval += integrationArray[i];
  } 
  
  return retval;
}

int getITerm(int error) {
    
  integrationArray[integrationPtr] = error;
  integrationPtr = (integrationPtr + 1) % INTEG_SIZE;
  
  int ITerm = I * integrate(); 
  
  if(ITerm > outMax) ITerm = outMax;
  else if(ITerm < outMin) ITerm = outMin;
  
  return ITerm;
}

int getPID(int value) {
  int error = value - setpoint;
  int angle = 90;
  int pidValue = 0;
  float dInput = 0;
  int ITerm = getITerm(error);
  
  dInput = error - lastInput;
  lastInput = error;
  
  pidValue = P * error +  ITerm + (int) (D * dInput);
  
  if(pidValue > outMax) pidValue = outMax;
  else if(pidValue < outMin) pidValue = outMin;
  
  return pidValue;
}

void outputLightPosition(int value) {
  if(value < -6) {
    Serial.println("-");
  } else if(value < -3) {
    Serial.println("  -"); 
  } else if(value < 0) {
     Serial.println("   -");
  } else if(value > 6) {
     Serial.println("         -");
  } else if(value > 3) {
     Serial.println("       -");
  } else {
     Serial.println("     -");
  } 
}


void follow(int pidValue) {
  int val = pidValue / scaleFactor;
  val += servoCenter;  
  
  
  if(val == servoCenter && servo.attached()) {
    servo.detach();
  } else {
    if(!servo.attached()) {
      servo.attach(servoPin); 
    }
    Serial.println(val);
    servo.write(val);
  }
}
