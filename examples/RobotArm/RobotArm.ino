/*
*      uStepper Robot Arm example sketch!
*
*
* This sketch requires the interconnection of 3 uSteppers. Two of them will be configured as Slaves and one of them as Master.
* A Master and Slave function is provided, comment in the appropriate one in the loop.
* It is recommended to use a wireless UART module to interface with the Master. This will eliminate the risk of damaging the USB port when the Robot Arm moves.
* 
* The pin configuration can be changed as desired (pin 6 and 7 is used for i2c though, so don't use these)
* 
* Pin connections:
* -------------------------------------
* |   Master  |         Slave          |
* |------------------------------------|
* | 5         |  0 (RX on both slaves) |
* | 3         |  2 (slave 1)           |
* | 8         |  2 (slave 2)           |
* | GND       |  GND                   |
* -------------------------------------
*
* The master will receive commands on what to do from its UART. It will then transmit the necessary commands to the Slaves through the software serial port on pin 4.
* The slaves will perform their operation and respond with a digital signal from their pin 2, which is received on pin 3 or 8 on the Master (depending on which slave responds).
* 
* The commands accepted by the master are as follows:
*
*  R  ->   Record position
*  P  ->   Play recorded sequence
*  S  ->   Stop
* 
* To record a sequence, press R. At every desired recording position, press R again. The code will record the position at each instant where R is pressed. To stop recording press S. To play the sequence, press P.
* The sequence can be run again by pressing P, and a new sequence can be recorded by pressing R.
* 
* For now, the code records the position and plays it without feedback during runtime. This will be implemented in a future release.
* If the motors are connected wrong (i.e. the connector is reversed compared to the arm this coe was written for) the Robot Arm will act strange! BE AWARE of this on the forst run!
* 
* Please be aware that the Robot Arm may do moves that can damage the arm itself or its surroundings. By using this code, you acknowledge that the authors can not be held responsible for damage.
* 
*/

#include <uStepper.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(4, 5); // RX, TX

//acceleration and velocity can be changed to fit your needs
#define MAXACCELERATION 1500         //Max acceleration = 1500 Steps/s^2
#define MAXVELOCITY 800           //Max velocity = 1100 steps/s

uint8_t record = 0, play = 0, stopped = 0;
uint16_t place = 0, endmove = 0;
int16_t pos[100];//Array for storing the positions recorded
bool finalMove = 0;
  
uStepper stepper(MAXACCELERATION, MAXVELOCITY);//Setup uStepper with the parameters defined

void setup() {
  stepper.setup();
  Serial.begin(9600);//Setup serial communication on TX/RX
  mySerial.begin(9600);
  pinMode(2, OUTPUT);//Output on slaves
  pinMode(3, INPUT);//Input from slave 1
  pinMode(8, INPUT);//Input from slave 2
  //Menu for serial interface
  Serial.println("CMD   Function");
  Serial.println("-------------------------------------------");
  Serial.println("R     Record position");
  Serial.println("P     Play recorded sequence");
  Serial.println("S     Stop");
  Serial.println();
}

void master()//Code running on Master
{
  char cmd;
  float error = 0;
  if(play ==1 && finalMove == 1)//If we are running the sequence and we are at our final move
  {
    while(digitalRead(3)==LOW || digitalRead(8)==LOW);//Wait for the two slaves to complete their moves
    while(stepper.getMotorState());//Wait for the master to complete its move
    cmd = 'S';//Send a stop command
    finalMove = 0;//We have done the final move
  }
  else if(play ==1 && place <= endmove)//If we are running the sequence but are not at the final move
  {
    while(digitalRead(3)==LOW || digitalRead(8)==LOW);//Wait for the slaves to complete their moves
    while(stepper.getMotorState());//Wait for the Master to complete its move
    cmd = 'F';//Send F to run the next step in the sequence
  }
  redo://A goto (yes it is ugly coding)
  if(play == 0)//If we are not running the sequence
  {
    while(!Serial.available());//Wait for something to come on the serial line
    cmd = Serial.read();//save it in the cmd variable
  }
  switch(cmd){//Switch according to received command
    case 'P'://play squence
      if(record == 1)//If we are recording
      {
        Serial.println("Press STOP to stop recording");
        goto redo;//goto redo mark
      }
      else//If not recording
      {
        play = 1;
        break;
        
      }
      
    case 'R'://record steps
    if(record == 0)//If we were not recording before
    {
      stepper.encoder.setHome();//Set current position as home
      place = 0;//Reset the array counter
      record = 1;//Record flag
      stepper.softStop(SOFT);//Stop the stepper without breaks
      mySerial.println('R');//Send record command to Slaves
      Serial.println("Press RECORD to record position - Press STOP to stop recording");
    }
    else if(record == 1)//If we have initialized recording
    {
      mySerial.println('R');//Send record command to slaves
      Serial.print("Position ");//Print position number for convenience
      Serial.print(place);
      Serial.println(" recorded");
      pos[place] = (int16_t)stepper.encoder.getAngleMoved();//Save current position
      place++;//Increment array counter
    }
      break;
      
    case 'F'://If we are stepping forward (in a play sequence scenario)
      stopped = 0;//Stopped flag = 0
      if(place == endmove)//If we are at the end move
      {
        place = 0;//reset array counter
        finalMove = 1;//set final move flag
      }
      mySerial.println('F');//send step forward command to Slaves
      Serial.print("Moving to position ");//Print where we are going for convenience
      Serial.println(place);
      error = ((int16_t)stepper.encoder.getAngleMoved()-pos[place]);//Do some open loop movement based on current position and position recorded.
      if(error < 0)
      {
        stepper.moveSteps((uint16_t)(-error*8.888889), CCW, HARD);
      }
      else if(error > 0)
      {
        stepper.moveSteps((uint16_t)(error*8.888889), CW, HARD);
      }
      else if(error == 0)
      {
        //
      }
      place++;//increment array counter
      break;
      
    case 'S'://1 press on stop => hard stop, second press => soft stop
      mySerial.println('S');
      if(record == 1)//if we are in recording state
      {
        endmove = place;//set the endmove to the current position
        place = 0;//reset array counter
        record = 0;//reset record flag
        stopped = 0;//reset stopped flag
      }
      play = 0;//reset play sequence flag
      if(stopped == 0)//if stop flag is 0
      {
        Serial.println();
        Serial.println("Stopped with breaks - send STOP again to release breaks");
        stepper.softStop(HARD);//stopped with brakes
        stopped = 1;//set stop flag to 1
      }
      else
      {
        Serial.println("Stopped without breaks");
        Serial.println();
        stepper.softStop(SOFT);//if stopflag was 1 and stop command was sent, deactivate brakes
      }      
      break;
      
    default:
    
      break;
  }
}

void slave()//Slave function - the functionality is pretty much the same as in the Master, so look at the comments there for reference.
{
  while(stepper.getMotorState());//if stepper is running
  {
    digitalWrite(2,LOW);//set pin 2 low
  }
  digitalWrite(2,HIGH);//if stepper is not running set pin 2 high
  
  char cmd;
  float error = 0;
  while(!Serial.available());
  cmd = Serial.read();//receive commands from the Master
  switch(cmd){
    case 'R':
      if(record == 0)
      {
        stepper.encoder.setHome();
        place = 0;
        record = 1;
        stepper.softStop(SOFT);
      }
      else if(record == 1)
      {
        pos[place] = (int16_t)stepper.encoder.getAngleMoved();//
        place++;
      }
      break;
      
    case 'F':
      stopped = 0;
      if(place == endmove)
      {
        place = 0;
      }
      digitalWrite(2,LOW);
      error = ((int16_t)stepper.encoder.getAngleMoved()-pos[place]);
      if(error < 0)
      {
        stepper.moveSteps((uint16_t)(-error*8.888889), CCW, HARD);
      }
      else if(error>0)
      {
        stepper.moveSteps((uint16_t)(error*8.888889), CW, HARD);
      }
      else if(error == 0)
      {
        //
      }
      place++;
      break;
      
    case 'S':
      if(record == 1)
      {
        endmove = place;
        place = 0;
        record = 0;
        stopped = 0;
      }
      if(stopped == 0)
      {
        stepper.softStop(HARD);
        stopped = 1;
      }
      else
      {
        stepper.softStop(SOFT);
      }  
      break;
      
    default:
    
      break;
  }
}

void loop() {//comment in the appropriate function
master();
//slave();
}
