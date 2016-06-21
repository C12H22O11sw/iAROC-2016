#include "robot.h"
#define MOVINGSPEED 50
#define LEFT_RIGHT_DIFF (1)
#define FRONT_BACK_DIFF (-2)


enum directions {
  NORTH, SOUTH, EAST, WEST, ROTATION
};
int robotDirection = EAST;

int robotXposition; //EAST WEST front back
int robotYposition; //NORTH SOUTH right left
int pathLength = 0;
String path [1];
boolean hasBeenHere [25][17];


/* quick reference
 *
 *  robot moving speed = 100 PWM
 *  X is the long axis on the field
 *  Y is the short axis on the field
 *  the robot always faces east
 *
 *
 *
 *
 *
 *
 */
//int SPEED = 50; //Speed constant for testing
//boolean[][] mazeMap = new boolean[25][9]; //[x][y]

//Walls: North, East, South, West

void addToPath(int x, int y)
{
  String coordinate = x + " " + y;
  pathLength++;
  int coordinateNumber = pathLength;

  //String pathHolder [path.length + 1];
}

void shortenPath()
{

}

//************************PING SENSOR METHODS ***************************//
//Unified distance method
long getDistanceValue(int dir)
{
  long duration, distance;
  int trigPin, echoPin;
  switch (dir) {
    case EAST:
      trigPin = frontTrigPin;
      echoPin = frontEchoPin;
      break;
    case WEST:
      trigPin = backTrigPin;
      echoPin = backEchoPin;
      break;
    case NORTH:
      trigPin = rightTrigPin;
      echoPin = rightEchoPin;
      break;
    case SOUTH:
      trigPin = leftTrigPin;
      echoPin = leftEchoPin;
      break;
    case ROTATION:
      trigPin = rotationTrigPin;
      echoPin = rotationEchoPin;
      break;
  }
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 5.59;

  return distance;
}

long getRightDistanceValue()
{
  return getDistanceValue(NORTH);
}
long getLeftDistanceValue()
{
  return getDistanceValue(SOUTH);
}

long getFrontDistanceValue()
{
  return getDistanceValue(EAST);
}
long getBackDistanceValue()
{
  return getDistanceValue(WEST);
}
long getRotationDistanceValue()
{
  return getDistanceValue(SOUTH) - getDistanceValue(ROTATION);
}

bool isWallOn(int direction) {
  return getDistanceValue(direction) < distanceFromWall;
}

bool isWallOnRight() {
  if (getRightDistanceValue() < distanceFromWall) {

    return true;
  }
  else {

    return false;
  }
}
bool isWallOnLeft() {
  if (getLeftDistanceValue() < distanceFromWall) {

    return true;
  }
  else {

    return false;
  }
}
bool isWallOnFront() {
  if (getFrontDistanceValue() < distanceFromWall) {

    return true;
  }
  else {

    return false;
  }
}
bool isWallOnBack() {
  if (getBackDistanceValue() < distanceFromWall) {

    return true;
  }
  else {

    return false;
  }
}

void testPingSensors() {
  if (getRightDistanceValue() == 0 || getLeftDistanceValue() == 0 || getFrontDistanceValue() == 0 || getBackDistanceValue() == 0 || getRotationDistanceValue() == 0) {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);              // wait for a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);
  }
  Serial.print("  R = ");
  Serial.print(getRightDistanceValue());
  Serial.print("  L = ");
  Serial.print(getLeftDistanceValue());
  Serial.print("  F = ");
  Serial.print(getFrontDistanceValue());
  Serial.print("  B = ");
  Serial.print(getBackDistanceValue());
  Serial.print("  RO-L = ");
  Serial.println(getRotationDistanceValue() - getLeftDistanceValue());
}
//************************MOTOR METHODS ***************************//

void setMotorSpeed(char motor, int velocity) {
  int motorPWM = motor * 2 + 4;
  int motorDIR = motor * 2 + 5;
  analogWrite(motorPWM, abs(velocity));//0 =0% 255= 100%
  if (velocity > 0) {
    digitalWrite(motorDIR, HIGH);//current flow from A - TO - B
  }
  else {
    digitalWrite(motorDIR, LOW);//current flow from B - TO - A
  }
}

void setMotorSpeeds(int backLeft, int backRight, int frontLeft, int frontRight)
{
  setMotorSpeed(backLeftMotor, backLeft);
  setMotorSpeed(backRightMotor, -backRight);// this motor is inverted becuase of a wireing issue.
  setMotorSpeed(frontLeftMotor,  frontLeft);
  setMotorSpeed(frontRightMotor, frontRight);
}
int reSize(int n, int limit) {
  if (n > limit) {
    return limit;
  }
  if (n < -limit) {
    return -limit;
  }
  else {
    return n;
  }

}
void setRobotSpeed(int x, int y, int r)
{
  x = reSize(x , MOVINGSPEED);
  y = reSize(y , MOVINGSPEED);
  r = reSize(r , MOVINGSPEED);
  int left_adjust = 5;
  int frontLeft = x + y * PI / 2 - r;
  int frontRight = -x + y * PI / 2 - r;
  int backLeft = -x + y * PI / 2 + r;
  int backRight = x + y * PI / 2 + r;
  setMotorSpeeds(backLeft, backRight, frontLeft, frontRight);
}

int distance = 87500 / MOVINGSPEED;

void MoveRobotOneBlock(int dir) {
  switch (dir) {
    case NORTH:
      setRobotSpeed(0, MOVINGSPEED, LEFT_RIGHT_DIFF);
      robotYposition ++;
      break;
    case SOUTH:
      setRobotSpeed(0, -MOVINGSPEED, -LEFT_RIGHT_DIFF);
      robotYposition --;
      break;
    case EAST:
      setRobotSpeed(-MOVINGSPEED, 0, -FRONT_BACK_DIFF);
      robotXposition --;
      break;
    case WEST:
      setRobotSpeed(MOVINGSPEED, 0, FRONT_BACK_DIFF);
      robotXposition ++;
      break;
  }
  int i;
  int time_step = distance / 7;
  for(i=0; i<7  ; i++) {
    if(getDistanceValue(dir) < 250) {
      break;
    }
    
    delay(time_step);
  }
  setRobotSpeed(0, 0, 0);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (int I = 5; I < 14; I++) {// go through ports 6-13 and set them to output
    pinMode(I, OUTPUT);//PWM is done through analog write
  }
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(backTrigPin, OUTPUT);
  pinMode(backEchoPin, INPUT);
  pinMode(rotationTrigPin, OUTPUT);
  pinMode(rotationEchoPin, INPUT);

}
int PID(int currentValue, int targetValue) {
  return currentValue - targetValue;
}
boolean robotIsalignedY() {
  boolean needsAlign = false;
  if (isWallOnRight() && abs(PID(getRightDistanceValue(), 173)) > 50) {
    needsAlign = true;
    //Serial.println("Needs align: Right");
  }
  else if (isWallOnLeft() && abs(PID(getLeftDistanceValue(), 173)) > 50) {
    needsAlign = true;
    //Serial.println("Needs align: Left");
  }
  return !needsAlign;
}
boolean robotIsalignedX() {
  boolean needsAlign = false;
  if (isWallOnFront() && abs(PID(getFrontDistanceValue(), 173)) > 50) {
    needsAlign = true;
    //Serial.println("Needs align: Front");
  }
  else if (isWallOnBack() && abs(PID(getBackDistanceValue(), 173)) > 50) {
    needsAlign = true;
    //Serial.println("Needs align: Back");
  }
  return !needsAlign;
}

boolean robotIsalignedR() {
  boolean needsAlign = false;
  if (isWallOnLeft() && abs(getRotationDistanceValue()) > 10) {
    needsAlign = true;
    //Serial.println("Needs align: Right");
  }

  return !needsAlign;
}




void align() {
  
  int x = 0;
  int y = 0;
  int r = 0;
  int startTime = millis();
  int targetDistanceFromWall = 143;
  int threshold = 430;
  while (!robotIsalignedY()) {
    //** Y correction **//
    if (getRightDistanceValue() < threshold) {
      y = -PID(targetDistanceFromWall, getRightDistanceValue());
      Serial.print("Aligning Right y = ");
      Serial.println(y);
    }
    else if (getLeftDistanceValue() < threshold) {
      y =   PID(targetDistanceFromWall, getLeftDistanceValue());
      Serial.print("Aligning Left y = ");
      Serial.println(y);
    }
    else {
      break;
    }
    setRobotSpeed(0 , y, 0);
  }
  while (!robotIsalignedX()) {
    //** X correction //

    if (getFrontDistanceValue() < threshold) {
      x = PID(targetDistanceFromWall, getFrontDistanceValue());
      Serial.print("Aligning Front x = ");
      Serial.println(x);
    }
    else if (getBackDistanceValue() < threshold) {
      x = -PID(targetDistanceFromWall, getBackDistanceValue());
      Serial.print("Aligning Back x = ");
      Serial.println(x);

    }
    else {
      break;
    }
    setRobotSpeed(x , 0, 0);
  }
  while (!robotIsalignedR()) {

    if (getRotationDistanceValue() < 360 && getLeftDistanceValue() < 360) {
      r = getRotationDistanceValue();
      Serial.print("Aligning Rotation r = ");
      Serial.println(r);
      setRobotSpeed(0 , 0, r);
    }
    else {
      break;
    }
    //y = 0;
    //r = 0;
    //x = 0;
    //Serial.println(r);

  }
  setRobotSpeed(0, 0, 0);
}

//*********************** LOOP METHOD ***************************//

void turnLeft() {
  switch (robotDirection) {
    case NORTH:
      robotDirection = EAST;
      break;
    case EAST:
      robotDirection = SOUTH;
      break;
    case SOUTH:
      robotDirection = WEST;
      break;
    case WEST:
      robotDirection = NORTH;
      break;
  }
}

void turnRight() {
  switch (robotDirection) {
    case NORTH:
      robotDirection = WEST;
      break;
    case EAST:
      robotDirection = NORTH;
      break;
    case SOUTH:
      robotDirection = EAST;
      break;
    case WEST:
      robotDirection = SOUTH;
      break;
  }
}

void print_direction(int dir) {
  switch (dir) {
    case NORTH:
      Serial.println("north");
      break;
    case SOUTH:
      Serial.println("south");
      break;
    case EAST:
      Serial.println("east");
      break;
    case WEST:
      Serial.println("west");
      break;
    default :
      Serial.println("ERROR");
      break;
  }
}
void loop() {

   align();
   setRobotSpeed(0, 0, 0);
   turnLeft();
   while (isWallOn(robotDirection)) {
     turnRight();
   }

   hasBeenHere[robotXposition][robotYposition] = true;

   print_direction(robotDirection);
   MoveRobotOneBlock(robotDirection);



   //testPingSensors();
  /*for (int i = 0; i < 3; i++) {
    MoveRobotOneBlock(EAST);
  }
  for (int i = 0; i < 3; i++) {
    MoveRobotOneBlock(WEST);
  }*/

}



