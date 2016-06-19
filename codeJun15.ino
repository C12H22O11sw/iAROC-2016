#define frontRightMotor 2
#define frontLeftMotor 1
#define backRightMotor 3
#define backLeftMotor 4

#define frontRightMotorPinPWM  8
#define frontRightMotorPinDIR  9
#define frontLeftMotorPinPWM  6
#define frontLeftMotorPinDIR  7
#define backRightMotorPinPWM  10
#define backRightMotorPinDIR  11
#define backLeftMotorPinPWM  12
#define backLeftMotorPinDIR  13



#define leftTrigPin 52
#define leftEchoPin 53
#define rightTrigPin 22
#define rightEchoPin 23
#define frontTrigPin 48
#define frontEchoPin 49
#define backTrigPin 46
#define backEchoPin 47

#define rotationTrigPin 52
#define rotationEchoPin 53
#define rotationTrigPin 50
#define rotationEchoPin 51

#define leftDistanceArray 1
#define rightDistanceArray 2
#define backDistanceArray 3
#define frontDistanceArray 4
#define rotationDistanceArray 5
#define distanceFromWall 350

int frontLeft;
int frontRight;
int backLeft;
int backRight;
enum directions {
  north, south, east, west
};
int robotDirection = east;
int robotFutureDirection = east;
int lastChangeInDirection = 0;

int robotXposition; //east west front back
int robotYposition; //north south right left
boolean hasBeenHere [25][9];
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

//************************PING SENSOR METHODS ***************************//
//Unified distance method
long getDistanceValue(int direction)
{
  long duration, distance;
  int trigPin, echoPin;
  switch (direction) {
    case east:
      trigPin = frontTrigPin;
      echoPin = frontEchoPin;
      break;
    case west:
      trigPin = backTrigPin;
      echoPin = backEchoPin;
      break;
    case north:
      trigPin = rightTrigPin;
      echoPin = rightEchoPin;
      break;
    case south:
      trigPin = leftTrigPin;
      echoPin = leftEchoPin;
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
  return getDistanceValue(north);
}
long getLeftDistanceValue()
{
  return getDistanceValue(south);
}

long getFrontDistanceValue()
{
  return getDistanceValue(east);
}
long getBackDistanceValue()
{
  return getDistanceValue(west);
}
long getRotationDistanceValue()
{
  long duration, distance;
  digitalWrite(rotationTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(rotationTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(rotationTrigPin, LOW);
  duration = pulseIn(rotationEchoPin, HIGH);
  distance = duration / 5.59;

  return distance;
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
  Serial.print("  RO = ");
  Serial.println(getRotationDistanceValue());
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
void setRobotSpeed(int x, int y, int r)
{
  constrain(x, -50, 50);
  constrain(y, -50, 50);
  constrain(r, -30, 30);
  frontLeft = x + y * PI / 2 - r;
  frontRight = -x + y * PI / 2 - r;
  backLeft = -x + y * PI / 2 + r;
  backRight = x + y * PI / 2 + r;
  setMotorSpeeds(backLeft, backRight, frontLeft, frontRight);
}
void MoveRobotNorthOneBlock() {
  setRobotSpeed(0, 50, 0);
  robotYposition ++;
  delay(1920);
}
void MoveRobotSouthOneBlock() {
  setRobotSpeed(0, -50, 0);
  robotYposition --;
  delay(1920);
}
void MoveRobotEastOneBlock() {
  setRobotSpeed(-50, 0, 0);
  robotXposition --;
  delay(1920);
}
void MoveRobotWestOneBlock() {
  setRobotSpeed(50, 0, 0);
  robotXposition ++;
  delay(1920);
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
boolean robotIsAlligned() {
  int nCorrectValues = 0;
  if (isWallOnRight() && abs(PID(getRightDistanceValue(), 173)) > 50) {
    nCorrectValues--;
  }
  if (isWallOnLeft() && abs(PID(getLeftDistanceValue(), 173)) > 50) {
    nCorrectValues--;
  }
  if (isWallOnFront() && abs(PID(getFrontDistanceValue(), 173)) > 50) {
    nCorrectValues--;
  }
  if (isWallOnBack() && abs(PID(getBackDistanceValue(), 173)) > 50) {
    nCorrectValues--;
  }
  Serial.println(nCorrectValues);
  if (nCorrectValues > -1) {
    Serial.println("true");
    return true;

  }
  else {
    Serial.println("false");
    return false;
  }
}



void allign() {
  int x = 0;
  int y = 0;
  int r = 0;
  int startTime = millis();
  int targetDistanceFromWall = 173;

  while (!robotIsAlligned()) {
    //** Y correction **//
    if (getRightDistanceValue() < 260) {
      y = -PID(targetDistanceFromWall, getRightDistanceValue());
    }
    else if (getLeftDistanceValue() < 260) {
      y =   PID(targetDistanceFromWall, getLeftDistanceValue());
    }

    //** X correction **//

    if (getFrontDistanceValue() < 260) {
      x = PID(targetDistanceFromWall, getFrontDistanceValue());
    }
    else if (getBackDistanceValue() < 260) {
      x = -PID(targetDistanceFromWall, getBackDistanceValue());
    }


    if (getRotationDistanceValue() < 360 && getLeftDistanceValue() < 360) {
      r = PID(getRotationDistanceValue(), getLeftDistanceValue());
    }
    //y = 0;
    //r = 0;
    //x = 0;
    setRobotSpeed(x/2 ,y/2,-r);
  }
}

//*********************** LOOP METHOD ***************************//

void turnLeft() {
  switch(robotDirection) {
    case north:
      robotDirection = east;
      break;
    case east:
      robotDirection = south;
      break;
    case south:
      robotDirection = west;
      break;
    case west:
      robotDirection = north;
      break;
  }
}

void turnRight() {
  switch(robotDirection) {
    case north:
      robotDirection = west;
      break;
    case east:
      robotDirection = north;
      break;
    case south:
      robotDirection = east;
      break;
    case west:
      robotDirection = south;
      break;
  }
}

void loop() {

  robotIsAlligned();
  testPingSensors();

  turnLeft();

  while(isWallOn(robotDirection)) {
    turnRight();
  }

  //if (robotDirection == north) {
    //if (!isWallOnFront()) {
      //robotDirection = east;
    //}//if the block east is empty
    //else if (!isWallOnRight()) {
      //robotDirection = north;
    //}
    //else if (!isWallOnBack()) {
      //robotDirection = west;
    //}
    //else if (!isWallOnLeft()) {
      //robotDirection = south;
    //}
  //} else if (robotDirection == east) {

    //if (!isWallOnLeft()) {
      //robotDirection = south;
    //}

    //else if (!isWallOnFront()) {
      //robotDirection = east;
    //}//north
    //else if (!isWallOnRight()) {
      //robotDirection = north;
    //} else if (!isWallOnBack()) {
      //robotDirection = west;
    //}
  //} else if (robotDirection == south) {

    //if (!isWallOnBack()) {
      //robotDirection = west;
    //}
    //else if (!isWallOnLeft()) {
      //robotDirection = south;
    //}
    //else if (!isWallOnFront()) {
      //robotDirection = east;
    //}
    //else if (!isWallOnRight()) {
      //robotDirection = north;//north
    //}
  //} else if (robotDirection == west) {
    //if (!isWallOnRight()) {
      //robotDirection = north;
    //}
    //else if (!isWallOnBack) {
      //robotDirection = west;
    //}
    //else if (!isWallOnLeft()) {
      //robotDirection = south;
    //} else if (!isWallOnFront()) {
      //robotDirection = east;
    //}
  //}

  //if (millis() - lastChangeInDirection > 2890) {
  lastChangeInDirection = millis();
  setRobotSpeed(0, 0, 0);
  //allign();
  int robotMovoingSpeed = 40;
  hasBeenHere[robotXposition][robotYposition] = true;
  switch (robotDirection) {
    case north:
      Serial.println("north");
      setRobotSpeed(0, 0, 0);
      allign();
      MoveRobotNorthOneBlock();
      break;
    case south:
      Serial.println("south");
      setRobotSpeed(0, 0, 0);
      allign();
      MoveRobotSouthOneBlock();
      break;
    case east:
      Serial.println("east");
      setRobotSpeed(0, 0, 0);
      allign();
      MoveRobotEastOneBlock();
      break;
    case west:
      Serial.println("west");
      setRobotSpeed(0, 0, 0);
      allign();
      MoveRobotWestOneBlock();
      break;
    default :
      Serial.println("ERROR");
      break;

  }
  //}
  //allign();
  



}
