  // GPS - Neo-6M
  #include <TinyGPS++.h>
  #include <SoftwareSerial.h>
  static const int RXPin = 19, TXPin = 18;
  static const uint32_t GPSBaud = 9600;

  // The TinyGPS++ object
  TinyGPSPlus gps;
  // The serial connection to the GPS device
  SoftwareSerial ss(RXPin, TXPin);

  // Traffic Lights - LED Outputs
  #define ledRed A0
  #define ledYellow A1 
  #define ledGreen A2

  // DC Motor & Motor Module - L298N
  #include <L298N.h>

  // Pin definition
  const unsigned int IN3 = 9;
  const unsigned int IN4 = 10;
  const unsigned int EN = 11;

  // Create one motor instance
  L298N motor(EN, IN3, IN4);

  // Servo
  #include <Servo.h>
  Servo myservo;

  //Potentiometer
  #define pot A3

  // Piezo Buzzer
  #define piezoPin A4

  // Sonar - HC-SR04
  #define echoPin 4 // attach pin D2 Arduino to pin Echo of HC-SR04
  #define trigPin A5 //attach pin D3 Arduino to pin Trig of HC-SR04

  // Line Sensor
#define lineSensorPin 7

// Crash Sensor / Button
#define crashSensor 8

void setup() {
  Serial.begin(9600);
// GPS
ss.begin(GPSBaud);

// Traffic Lights - LED Outputs
pinMode(ledRed, OUTPUT);
pinMode(ledYellow, OUTPUT);
pinMode(ledGreen, OUTPUT);

// DC Motor & Motor Module - L298N
motor.setSpeed(70);

// Servo
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  //Potentiometer
pinMode(pot, INPUT);

// Piezo Buzzer
pinMode(piezoPin,OUTPUT);

// Sonar - HC-SR04
pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

// Line Sensor
pinMode(lineSensorPin, OUTPUT);

// Crash Sensor / Button
pinMode(crashSensor, INPUT);
}


void loop() {
  //
}
