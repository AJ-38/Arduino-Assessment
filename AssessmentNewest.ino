#include <Wire.h>
#include <Zumo32U4.h>
#include <StackArray.h>
#include <Vector.h>
#include <Timer.h>

#define QTR_THRESHOLD     450
#define REVERSE_SPEED     -150 // originally 150
#define TURN_SPEED        150 // originally 150
#define FORWARD_SPEED     100 // originally 150
#define REVERSE_DURATION  150 // originally 150
#define TURN_DURATION     150 // originally 150
#define OBSTACLE_THRESHOLD 11 //Threshold value for proximity sensor reading
#define HOUSE_DETECTED    true // Boolean value to indicate house detected
#define HOUSE_FOUND_DELAY  3000 // delay after house found 



Zumo32U4LineSensors linesensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4ProximitySensors proxSensors; // Declaration of prox Sensors

Timer timer;


int StorageArray[50];
int TimeStorageArray[50];

Vector<int> turnMemory;
Vector<int> turnMemoryTime;

struct MovementAction {
  int action; // Action identifier, move forward, left, right or reverse
  int duration; // Duration of action within milliseconds
};

 
void turnRight(){
  motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
  delay(TURN_DURATION);
}

void turnLeft(){
  motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
  delay(TURN_DURATION);
}

void moveForward(){
  motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  
}

void reverse(){
  motors.setSpeeds(REVERSE_SPEED, REVERSE_SPEED);
  delay(REVERSE_DURATION);
  
}

#define NUM_SENSORS 3
unsigned int linesensorvalues[NUM_SENSORS];

const uint16_t maxSpeed = 400;


void calibrateSensors() 
{
 
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <=90)
    {
    motors.setSpeeds(-200, 200);
  }
  else
  {
    motors.setSpeeds(200, -200);
  }

  linesensors.calibrate();
  }
  motors.setSpeeds(0,0);
}



void setup() {
  
  
  linesensors.initThreeSensors();
  proxSensors.initThreeSensors();

  buzzer.play(">g32>>c32"); // plays a sound to let user know it is on

 

  buttonA.waitForButton();  // waits for user to press buttonA to being calibration of sensors

  calibrateSensors(); // calling the sensor calibration function


  buttonA.waitForButton(); // waits for button A to be pressed again before it navigates the maze
 
 
 turnMemory.setStorage(StorageArray,0);
 turnMemoryTime.setStorage(TimeStorageArray, 0);
 
}

void returnHome() { // This function takes into account all movemenets that have been made and then does the opposite of them for the same duration of time. The items are pushed and popped from the 2 names vectors.

  turnMemory.pop_back();
  switch(turnMemory[turnMemory.size() - 1]){
      case 0 : 
      turnMemory.pop_back();
      turnLeft();
      
      break;

      case 1:
      turnMemory.pop_back();
      turnRight();
      
      break; 
    
  }

    reverse();
    delay(turnMemoryTime[turnMemoryTime.size() - 1]);
  turnMemoryTime.pop_back();
  motors.setSpeeds(0,0);
}

void loop() {


  Serial.begin(9600);
  timer.start();


  proxSensors.read(); // takes the integer value that the sensors detect (Larger the number, the closer the item is)
  
  Serial.println("Sensor 1:");
  Serial.println(linesensorvalues[0]); // Prints the line sensor values so that we can identify if the sensors are working correctly and evenly on either side
  Serial.println("Sensor 2:");
  Serial.println(linesensorvalues[NUM_SENSORS -1]); // Prints the line sensor values so that we can identify if the sensors are working correctly and evenly on either side. [NUM_SENSORS - 1] is used as we have declared 3 sensors so it is choosing the right hand sensor which is also known as line sensor 2.

  
  
  // put your main code here, to run repeatedly:
  

 
 linesensors.read(linesensorvalues);
  int obstacleValue = proxSensors.countsFrontWithLeftLeds() + proxSensors.countsFrontWithRightLeds();
  
  Serial.print("ObstacleValue:");
  Serial.println(obstacleValue);  // the lines shown above are creating an integer variable that stores the proximity sensor value; this is so that we can use this threshold to avoid hitting the objects as we can create a loop with these values.

  // If an obstacle is detected within a certain threshold, stop and avoid it
  if (obstacleValue > OBSTACLE_THRESHOLD) // I have declared an obstacle threshold of 11; I originally chose this as when I put the object in front of my proximity sensors, it reads at 12.
  {
    buzzer.play(">g32>>c32"); // Play sound to indicate house detection
    motors.setSpeeds(0,0);
    
 returnHome();

    
    /* Reverse
    motors.setSpeeds(REVERSE_SPEED, REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(REVERSE_SPEED, REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION); */

    /* Turn randomly to the left or right to avoid the obstacle
    if (random(2) == 0) {
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED); // Turn right
    } else {
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED); // Turn left
    }
    delay(TURN_DURATION); // this would be used if the object was in the middle of the road to navigate around it, however as we have been informed the object is large and in a dead end, this will not be useful.
    */
    // Move forward again after avoiding the obstacle
    // motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else
  {
    
  

  
    // If no obstacle is detected, follow the line
  
    // If left line sensor detects a line, stop, reverse and turn right
    if (linesensorvalues[0] > QTR_THRESHOLD)
    {
      motors.setSpeeds(0,0);
      delay(100);

      reverse();
      
      turnMemoryTime.push_back(timer.read());
      timer.stop();


      

      

      turnRight();
      
      turnMemory.push_back(1);
      turnMemoryTime.push_back(timer.read());
      timer.stop();
      
      
      
      

      moveForward();
      turnMemoryTime.push_back(timer.read());
      timer.stop();
      
      

      

      
    }
  
 
    // If right line sensor detects a line, reverse and turn left
     if (linesensorvalues[2] > QTR_THRESHOLD)
    {
      motors.setSpeeds(0,0);
      reverse();
      turnMemoryTime.push_back(timer.read());
      timer.stop();

      

      

      turnLeft();
      turnMemory.push_back(0);
      turnMemoryTime.push_back(timer.read());
      timer.stop();
      
      
      moveForward();
      turnMemoryTime.push_back(timer.read());
      timer.stop();

      
    }

    if (linesensorvalues[0] > QTR_THRESHOLD && linesensorvalues[2] > QTR_THRESHOLD){  // if both the left and right sensor are greater than the threshold i.e. have both hit a black line, the zumo will perform the reverse and spin zumo to get it out of a corner
        
        motors.setSpeeds(0,0);
        reverse();
        turnMemoryTime.push_back(timer.read());
        timer.stop();

        
        turnRight();
        turnMemory.push_back(1);
        turnMemoryTime.push_back(timer.read());
        timer.stop();

        
        turnRight();
        turnMemory.push_back(1);
        turnMemoryTime.push_back(timer.read());
        timer.stop();

        
        turnRight();
        turnMemory.push_back(1);
        turnMemoryTime.push_back(timer.read());
        timer.stop();

        
        turnRight();
        turnMemory.push_back(1);
        turnMemoryTime.push_back(timer.read());
        timer.stop();

        
        moveForward();

        turnMemoryTime.push_back(timer.read());
        timer.stop();

        
      }
    
    else
    {
      // Move straight forward if no line is detected
      moveForward();

      turnMemoryTime.push_back(timer.read());
      timer.stop();
    }
  }
}



/* proxSensors.read();
      if (proxSensors.countsFrontWithLeftLeds() >= 2
        || proxSensors.countsFrontWithRightLeds() >= 2)
      {
        changeState(StateDriving);
      }

proxSensors.read();
    uint8_t sum = proxSensors.countsFrontWithRightLeds() + proxSensors.countsFrontWithLeftLeds();
    int8_t diff = proxSensors.countsFrontWithRightLeds() - proxSensors.countsFrontWithLeftLeds();



 motors.setSpeeds(forwardSpeed, forwardSpeed);

      if (proxSensors.countsLeftWithLeftLeds() >= 2)
      {
        // Detected something to the left.
        scanDir = DirectionLeft;
        changeState(StateScanning);
      }

      if (proxSensors.countsRightWithRightLeds() >= 2)
      {
        // Detected something to the right.
        scanDir = DirectionRight;
        changeState(StateScanning);
      }

      ledRed(0);
    }

     // We see something with the front sensor but it is not a
      // strong reading.

      if (diff >= 1)
      {
        // The right-side reading is stronger, so veer to the right.
        motors.setSpeeds(veerSpeedHigh, veerSpeedLow);
      }
      else if (diff <= -1)
      {
        // The left-side reading is stronger, so veer to the left.
        motors.setSpeeds(veerSpeedLow, veerSpeedHigh);
      }
      else
      {
        // Both readings are equal, so just drive forward.
        motors.setSpeeds(forwardSpeed, forwardSpeed);
      }
      ledRed(0);
    } */
