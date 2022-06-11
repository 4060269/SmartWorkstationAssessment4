//SD Card Global
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"
RTC_Millis rtc;     // Software Real Time Clock (RTC)
DateTime rightNow;  // used to store the current time.

//GPS Global
bool gpsconState = false; //global variable to reference for outputs
static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
TinyGPSPlus gps; // The TinyGPS++ object
SoftwareSerial ss(RXPin, TXPin);// The serial connection to the GPS device

//Lights Global
bool orangeeventValue = false; //global variable to reference for outputs

//Piezo Global
bool gbootSuccess = false; //global variable to reference for outputs

//Servo Global
bool gservoState = false; //global variable to reference for outputs
#include <Servo.h>
Servo servo;

//DC Global
bool gdcState = false; //global variable to reference for outputs
#include <L298N.h>
const unsigned int IN1 = 11; // Pins definition
const unsigned int IN2 = 12;
const unsigned int EN = 9;
L298N motor(EN, IN1, IN2); // Create one motor instance

//Sonar Global
int gsonarValue = 0; //global variable to reference for outputs
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

//Line Global
int glineState = 0; //global variable to reference for outputs

//Pot Global
int gpotValue = 0; //global variable to reference for outputs

//Button 1 Global
bool gbutton1State = false; //global variable to reference for outputs
int inPin = 8;
int valueCrash = 1;

//Button 2 Global
bool gbutton2State = false; //global variable to reference for outputs

//Button 3 Global
bool gbutton3State = false; //global variable to reference for outputs

void setup() {
  //SD Card Setup
  Serial.begin(9600);
  Serial.print("Initialising SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");


  // Real Time Clock (RTC)
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println("initialization done.");
  logEvent("System Initialisation...");

  //GPS Setup
  ss.begin(GPSBaud);

  //Lights Setup
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  //Piezo Setup
  pinMode(48, OUTPUT);

  //Servo Setup
  servo.attach(6);

  //DC Setup
  motor.setSpeed(255);

  //Sonar Setup
  pinMode(4, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(5, INPUT); // Sets the echoPin as an INPUT

  //Line Setup
  pinMode(7, INPUT);

  //Pot Setup

  //Button1Setup
  pinMode(8, OUTPUT);
  pinMode(inPin, INPUT_PULLUP);

  //Button2Setup
  pinMode(9, INPUT);

  //Button3Setup
  pinMode(10, INPUT);
}

void loop() {
  logEvent(String dataToLog);
  GPSLoop();
  lightsLoop();
  piezoLoop();
  servoLoop();
  DCLoop():
    sonarLoop();
    lineLoop();
    potLoop();
    button1Loop();
    button2Loop();
    button3Loop();
  }

  // SD Card Loop
  void logEvent(String dataToLog) {
  /*
     Log entries to a file on an SD card.
  */
  // Get the updated/current time
  DateTime rightNow = rtc.now();

  // Open the log file
  File logFile = SD.open("events.csv", FILE_WRITE);
  if (!logFile) {
    Serial.print("Couldn't create log file");
    abort();
  }

  // Log the event with the date, time and data
  logFile.print(rightNow.year(), DEC);
  logFile.print(",");
  logFile.print(rightNow.month(), DEC);
  logFile.print(",");
  logFile.print(rightNow.day(), DEC);
  logFile.print(",");
  logFile.print(rightNow.hour(), DEC);
  logFile.print(",");
  logFile.print(rightNow.minute(), DEC);
  logFile.print(",");
  logFile.print(rightNow.second(), DEC);
  logFile.print(",");
  logFile.print(dataToLog);

  // End the line with a return character.
  logFile.println();
  logFile.close();
  Serial.print("Event Logged: ");
  Serial.print(rightNow.year(), DEC);
  Serial.print(",");
  Serial.print(rightNow.month(), DEC);
  Serial.print(",");
  Serial.print(rightNow.day(), DEC);
  Serial.print(",");
  Serial.print(rightNow.hour(), DEC);
  Serial.print(",");
  Serial.print(rightNow.minute(), DEC);
  Serial.print(",");
  Serial.print(rightNow.second(), DEC);
  Serial.print(",");
  Serial.println(dataToLog);
}

//GPS Loop
void GPSLoop() {
  /*
     Gets the GPS coords and tests whether it's in "bounds"
     @params: none
     @return: void
  */

  void locationBarrier() {
    while (ss.available() > 0)
      if (gps.encode(ss.read()))
        getGPSInfo();
  }

  void getGPSInfo() {
    Serial.print(F("Location: "));
    if (gps.location.isValid()) {
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
    }
    else {
      Serial.println(F("INVALID"));
    }
  }

  //Lights Loop
  void lightsLoop() {
    digitalWrite(A1, HIGH);
    digitalWrite(A2, HIGH);
    digitalWrite(A3, HIGH);
    delay(1000);
    digitalWrite(A1, LOW);
    digitalWrite(A2, LOW);
    digitalWrite(A3, LOW);
    delay(1000);
  }

  //Piezo Loop
  void piezoLoop() {
    if (
      tone(48, 1000, 500);
  }

//Servo Loop
void servoLoop() {
    if
  }
servo.write(0);
  delay(500);
  delay(10000);
  servo.write(90);
  delay(500);
  delay(10000);
  servo.write(180);
  delay(500);
  delay(10000);
  servo.write(90);
  delay(500);
  delay(10000);
  servo.write(0);
  delay(500);
  delay(10000);

  //DC Loop
  void DCLoop() {
    motor.forward();
    delay(10000);
    motor.stop();
    delay(1000);
    motor.backward();
    delay(10000);
  }

  //Sonar Loop
  void sonarLoop() {

    // Clears the trigPin condition
    digitalWrite(4, LOW);
    delayMicroseconds(2);

    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(4, HIGH);
    delayMicroseconds(10);
    digitalWrite(4, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(5, HIGH);

    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

    // Displays the distance on the Serial Monitor
    Serial.print("Distance: ");
    delay(300);
    Serial.print(distance);
    delay(300);
    Serial.println(" cm");
  }

  //Line Loop
  void lineLoop() {
    bool value = digitalRead(7);
    if (value == 0) {
      Serial.println("Yay");
    }
  }

  //Pot Loop
  void potLoop() {
    int sensorValue = analogRead(A0); //read the input on analog pin 0
    Serial.println(sensorValue); //print out the value you read
    delay(300); //delay in between reads for stability
  }

  //Button1Loop
  void button1Loop() {
    valueCrash = digitalRead(inPin); // read the state of the crash sensor
    if (valueCrash == 0) {
      // Turn on Red LED
      digitalWrite(A1, HIGH);
    } else {
      digitalWrite(A1, LOW);
    }
  }

  //Button2Loop
  void button2Loop() {
    // read the state of the pushbutton value:
    buttonState = digitalRead(9);

    // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
    if (buttonState == 0) {
      // turn orange LED on:
      digitalWrite(A1, HIGH);
    } else {
      // turn LED off:
      digitalWrite(A1, HIGH);
    }
  }

  //Button3Loop
  void button3Loop() {
    // read the state of the pushbutton value:
    buttonState = digitalRead(10);

    // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
    if (buttonState == 0) {
      // turn green LED on:
      digitalWrite(A2, HIGH);
    } else {
      // turn LED off:
      digitalWrite(A2, LOW);
    }
  }
