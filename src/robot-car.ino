<<<<<<< Updated upstream
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

enum Motor { LEFT, RIGHT };


void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);

  pinMode(trigPin , OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite( trigPin , LOW);
  
  servo.attach( servoPin );
  servo.write(95);
  delay(200);

  // Scan the surroundings before starting
  servo.write( sensorAngle[0] );
  delay(200);
  for (unsigned char i = 0 ; i < NUM_ANGLES ; i ++)
    readNextDistance(), delay(200);

  servo.write(95);
  printDistances(); 

  //testMotors ();
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
  if (angleIndex == NUM_ANGLES -1) {
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

void printDistances(){
  for (unsigned char i = 0 ; i < NUM_ANGLES ; i ++) {
    Serial.print(distance[i]);
    Serial.print(", ");
  }
  Serial.println();
}

void loop() {
  readNextDistance ();
  
  // See if something is too close at any angle
  bool tooClose = 0;
  for (short i = 0 ; i < NUM_ANGLES ; i++) {
    if ( distance [ i ] < 300) tooClose = 1;
    printDistances();
  }
  
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
=======
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

enum Motor { LEFT, RIGHT };


void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);

  pinMode(trigPin , OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite( trigPin , LOW);
  
  servo.attach( servoPin );
  servo.write(95);
  delay(200);

  // Scan the surroundings before starting
  servo.write( sensorAngle[0] );
  delay(200);
  for (unsigned char i = 0 ; i < NUM_ANGLES ; i ++)
    readNextDistance(), delay(200);

  servo.write(95);
  printDistances(); 

  //testMotors ();
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
  if (angleIndex == NUM_ANGLES -1) {
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

void printDistances(){
  for (unsigned char i = 0 ; i < NUM_ANGLES ; i ++) {
    Serial.print(distance[i]);
    Serial.print(", ");
  }
  Serial.println();
}

void loop() {
  readNextDistance ();
  
  // See if something is too close at any angle
  bool tooClose = 0;
  for (short i = 0 ; i < NUM_ANGLES ; i++) {
    if ( distance [ i ] < 300) tooClose = 1;
    printDistances();
  }
  
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
>>>>>>> Stashed changes
