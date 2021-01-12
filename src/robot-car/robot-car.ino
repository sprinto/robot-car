#include <Servo.h>
#include <L298NX2.h>

Servo servo;

// Ultrasonic Module pins
const int trigPin = 12; // 10 microsecond high pulse causes chirp , wait 50 us
const int echoPin = 13; // Width of high pulse indicates distance

// Servo motor that aims ultrasonic sensor .
#define NUM_ANGLES 7
unsigned char sensorAngle[NUM_ANGLES] = { 65, 75, 85, 95, 105, 115, 120 };
unsigned int distance [NUM_ANGLES];
enum distanceCategory {TOO_CLOSE = 300, CLOSE = 500};
const int servoPin = 11; // PWM output for hobby servo

// Motor control pins : L298N H bridge
const unsigned int EN_A = 9;
const unsigned int IN1_A = 7;
const unsigned int IN2_A = 5;

const unsigned int IN1_B = 4;
const unsigned int IN2_B = 2;
const unsigned int EN_B = 3;

// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);
enum Speed {TURN_SPEED = 80, SLOW_SPEED = 100, NORMAL_SPEED = 150};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  while (!Serial) {}

  pinMode(trigPin , OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite( trigPin , LOW);

  servo.attach( servoPin );
  servo.write(95); // Position in center

  // Scan the surroundings before starting
  scanAllAngles();
  printDistances();

  // set up dc motors
  motors.setSpeedA(0);
  motors.setSpeedB(0);
  delay(200);
  testMotors ();
  delay(200);
goBackTurnLeft();
  delay(200);
  testMotors ();

delay(2000);
}

void testMotors ()
{
  int speed[6] = { 90, 128, 255, 0};

  motors.stopA();
  motors.stopB();
    printMotorInfo();

  for (short i = 0 ; i < 4 ; i++) {
    motors.setSpeedA(speed[i ]);
    motors.forwardA();
    delay(200);
    printMotorInfo();
  }

  for (short i = 0 ; i < 4 ; i++) {
    motors.setSpeedA(speed[i ]);
    motors.backwardA();
    delay(200);
    printMotorInfo();
  }

  motors.stopA();

  for (short i = 0 ; i < 4 ; i++) {
    motors.setSpeedB(speed[i ]);
    motors.forwardB();
    delay(200);
    printMotorInfo();
  }

  for (short i = 0 ; i < 4 ; i++) {
    motors.setSpeedB(speed[i ]);
    motors.backwardB();
    delay(200);
    printMotorInfo();
  }

  motors.stopB();
  printMotorInfo();

}

void goBackTurnLeft()
{
  motors.stopA();
  motors.stopB();
  Serial.println("-----------------");
    printMotorInfo();

  // back slowly
  Serial.println("backwards");
  motors.setSpeed(150);
  //motors.setSpeedB(150);
  motors.backward();
  //motors.backwardB();
      printMotorInfo();

  delay(2000);

  // turn left by slowing down one wheel
  Serial.println("turn");
  motors.setSpeedB(100);
  motors.backwardB();
      printMotorInfo();

  delay(1000);

  motors.stopA();
  motors.stopB();
  Serial.println("-----------------");

}

// Scan the area ahead by sweeping the ultrasonic sensor left and right
// and recording the distance observed. This takes a reading , then
// sends the servo to the next angle. Call repeatedly once every 50 ms or so.
void readNextDistance ()
{
  static unsigned char angleIndex = 0;
  static signed char step = 1;

  distance [angleIndex ] = readDistance ();
  angleIndex += step ;
  if (angleIndex == NUM_ANGLES - 1) {
    step = -1;
  }
  else if (angleIndex == 0) {
    step = 1;
  }
  servo.write(sensorAngle[angleIndex ]);
}

// Read distance from the ultrasonic sensor , return distance in mm
//
// Speed of sound in dry air , 20C is 343 m/s
// pulseIn returns time in microseconds (10ˆ−6)
// 2d = p * 10ˆ−6 s * 343 m/s = p * 0.00343 m = p * 0.343 mm/us
unsigned int readDistance ()
{
  digitalWrite ( trigPin , HIGH );
  delayMicroseconds (10);
  digitalWrite ( trigPin , LOW );
  unsigned long period = pulseIn ( echoPin, HIGH );
  return period * 343 / 2000;
}

// Scan all angles at start and when changing direction
void scanAllAngles() {
  servo.write( sensorAngle[0] );
  for (unsigned char i = 0 ; i < NUM_ANGLES ; i ++)
    readNextDistance(), delay(200);
}

void loop() {
  // in setup env has been scanned

  // See if something is too close at any angle
  bool tooClose = 0;
  bool closing = 0;
  for (short i = 0 ; i < NUM_ANGLES && !tooClose && !closing ; i++) {

    if (distance [ i ] < TOO_CLOSE) {
      tooClose = 1;
      Serial.println("Too CLOSE: STOP, BACK, TRY LEFT");
      printDistances();
      goBackTurnLeft();
      scanAllAngles();
      printDistances();

    }
    else if ( distance [ i ] < CLOSE) {
      closing = 1;
      motors.setSpeed(SLOW_SPEED);

      switch (i) {

        // Check center first
        case 2:
        case 3:
        case 4:
          Serial.println("Closing CENTER, slow down");
          break;

        case 0:
        case 1:
          Serial.println("Closing RIGHT, slow down go left");
          break;


        case 5:
        case 6:
          Serial.println("Closing LEFT, slow down go right");
          break;

        default:
          Serial.println("Unknown state stopping ...");
          motors.stop();
      }
    }
  }

  if (!tooClose && !closing) {
    Serial.println("GO FWD");
    motors.setSpeed(NORMAL_SPEED);
    motors.forwardA();
    motors.forwardB();
    readNextDistance ();
  } else {
    // Some change in direction done, scan environment
    scanAllAngles();
  }

  // printDistances();


  //  if (tooClose) {
  //    // Something's nearby: back up left
  //    go(LEFT, -180);
  //    go(RIGHT, -80);
  //  } else {
  //    // Nothing in our way: go forward
  //    go(LEFT, 200);
  //    go(RIGHT, 200);
  //  }

  // Check the next direction in 50 ms
  delay (50);
}

/*
  Print some informations in Serial Monitor
*/
void printDistances() {
  for (unsigned char i = 0 ; i < NUM_ANGLES ; i ++) {
    Serial.print(distance[i]);
    Serial.print(", ");
  }
  Serial.println();
}


void printMotorInfo()
{
  String dirA;
  String dirB;

  switch (motors.getDirectionA()) {
    case L298N::FORWARD:
      dirA = "FWD";
      break;
    case L298N::BACKWARD:
      dirA = "BACKWD";
      break;
    case L298N::STOP:
      dirA = "STOP";
      break;
  default: dirA = "UNKNOWN direction";      
  }
  
  switch (motors.getDirectionB()) {
    case L298N::FORWARD:
      dirB = "FWD";
      break;
    case L298N::BACKWARD:
      dirB = "BACKWD";
      break;
    case L298N::STOP:
      dirB = "STOP";
      break;
  default: dirB = "UNKNOWN direction";      
  }
  
  Serial.print("Motor A is moving = ");
  Serial.print(motors.isMovingA() ? "YES, " : "NO, ");
  Serial.print(dirA);
  Serial.print(" at speed = ");
  Serial.println(motors.getSpeedA());
  Serial.print("Motor B is moving = ");
  Serial.print(motors.isMovingB() ? "YES " : "NO ");
    Serial.print(dirB);
  Serial.print(" at speed = ");
  Serial.println(motors.getSpeedB());
}
