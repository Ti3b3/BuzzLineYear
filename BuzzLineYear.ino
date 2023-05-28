
#include  <QTRSensors.h>

/*************************************************************************
*  Sensor Array object initialisation 
*************************************************************************/
QTRSensors  qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

/*************************************************************************
*  PID control system variables 
*************************************************************************/
float  Kp = 0.055; //related to the proportional control term; 
         
float Ki = 0.00025; //related to the integral  control term; 
             
float  Kd = 0.7; //related to the derivative control term; 
             
int P;
int I;
int D;

/*************************************************************************
*  Global variables
*************************************************************************/
int  lastError = 0;
float lastPos = 0;
boolean line = false;
int keepGoing = 1500;
boolean onoff = false;
uint16_t position = 3500;
/*************************************************************************
*  Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
const  uint8_t maxspeeda = 105;
const uint8_t maxspeedb =  105;
const uint8_t basespeeda  = 68;
const uint8_t basespeedb = 68;

/*************************************************************************
*  L293 GPIO pins declaration
*************************************************************************/
// Motor A connections
int enA = 11;
int in1 = 2;
int in2 = 3;
// Motor B connections
int enB = 10;
int in3 = 4;
int in4 = 5;

void  setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, 6 , 7}, SensorCount);

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  delay(500);
  
  boolean Ok = false;
  
  while (Ok == false) { // the main function won't start  until the robot is calibrated
    
      calibration(); //calibrate the robot for 10 seconds
      Ok = true;
    
  }
  forward_brake(0, 0); //stop the motors
}

void  calibration() {
  for (uint16_t i = 0;  i < 400; i++)
  {
    qtr.calibrate();
  }
}

void  loop() {  

    position = qtr.readLineBlack(sensorValues); //read  the current position

    //Check if there is a sudden change if so continue driving forward
    if(((position > 6500) || (position < 500)) && ((lastPos < (3500 + keepGoing)) && (lastPos > (3500 - keepGoing))) && !line){
      line = true;
    }
    else if(line){
      forward_brake(basespeeda, basespeedb);
      if((position > 100) && (position < 6900)){
        line = false;
      }
    }
    else{
      //Else just use the PID controller
      PID_control();
    }
     lastPos = position;

}

void  forward_brake(int posa, int posb) {
  // Turn on motor A & B
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, posa);
  analogWrite(enB, posb);
}

void  PID_control() {
  
  int error = 3500 - position; //3500 is the ideal position  (the centre)

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki+ D*Kd; //calculate the correction
                                       //needed to be applied to the speed
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb -  motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0)  {
    motorspeedb = 0;
  } 
  forward_brake(motorspeeda, motorspeedb);
}
