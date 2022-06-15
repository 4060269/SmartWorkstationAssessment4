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
bool eventValue = false;
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

//Sonar Loop
long duration;
int distance;

//Line Global
int glineState = 0; //global variable to reference for outputs
bool gpsState = false;

//Pot Global
int gpotValue = 0; //global variable to reference for outputs

//Button 1 Global
bool gbutton1State = false; //global variable to reference for outputs
bool deactivateCircuit = false;
int inPin = 8;
int valueCrash = 1;

//Button 2 Global
bool gbutton2State = false; //global variable to reference for outputs
bool deactivateDc = false;

//Button 3 Global
bool gbutton3State = false; //global variable to reference for outputs
bool deactivateServo = false;

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
  Serial.println("initialization done.");  // logEvent("System Initialisation...");

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

  //Line Setup
  pinMode(7, INPUT);

  //Pot Setup
  pinMode(A0, OUTPUT);

  //Button1Setup
  pinMode(8, OUTPUT);
  pinMode(inPin, INPUT_PULLUP);

  //Button2Setup
  pinMode(9, INPUT);

  //Button3Setup
  pinMode(10, INPUT);
}

void loop() {
  //Button1Loop
  int valueCrash = digitalRead(inPin); // read the state of the crash sensor
  if (valueCrash == HIGH) {
    gbutton1State == true;
    deactivateCircuit == true;
  } else {
    gbutton1State == false;
    deactivateCircuit == false;
  }


  //Button2Loop
  int button2State = digitalRead(9);   // read the state of the pushbutton value:
  if (button2State == HIGH) {   // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
    gbutton2State == true;
    deactivateDc == true;
  } else {
    gbutton2State == false;
    deactivateDc == false;
  }


  //Button3Loop
  int button3State = digitalRead(10);   // read the state of the pushbutton value:
  if (button3State == HIGH) {   // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
    gbutton3State == true;
    deactivateServo == true;
  } else {
    gbutton3State == false;
    deactivateServo == false;
  }


  //Lights Loop
  if (eventValue == true) {
    digitalWrite(A3, HIGH);
    delay(5000);
    digitalWrite(A3, LOW);
    delay(6000);
  } else if (orangeeventValue == true) {
    digitalWrite(A2, HIGH);
    delay(5000);
    digitalWrite(A2, LOW);
  } else if (eventValue == false) {
    digitalWrite(A1, HIGH);
    delay(5000);
    digitalWrite(A1, LOW);
  }


  //Piezo Loop
  if (gbootSuccess == true) {
    tone(48, 1000, 500);
  } else if (gbootSuccess == false) {
    tone(24, 500, 250);
  }


  //Servo Loop
  if (gbutton3State == true) {
    eventValue == true;
    servo.write(0);
    delay(500);
    servo.write(90);
    delay(500);
    servo.write(180);
    delay(500);
  } else if (gbutton3State == false) {
    eventValue == false;
  }


  //DC Loop
  if (gbutton2State == true) {
    motor.forward();
    delay(10000);
    motor.stop();
  } else if (gbutton2State == false) {
    eventValue == false;
  }


  //Sonar Loop
  // Clears the trigPin condition
  digitalWrite(4, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(4, HIGH);
  delayMicroseconds(10);
  digitalWrite(4, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(5, HIGH);

  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  delay(3900);
  Serial.print(distance);
  delay(3900);
  Serial.println(" cm");



  //Line Loop
  bool value = digitalRead(7);
  if (value == 0) {
    Serial.println("Yay");


    //Pot Loop
    int sensorValue = analogRead(A0); //read the input on analog pin 0
    sensorValue = gpotValue;
    delay(300); //delay in between reads for stability
  }
}



  // SD Card Loop
  void logEvent(String dataToLog) {
    /*
       Log entries to a file on an SD card.
    */
    // Get the updated/current time
    DateTime rightNow = rtc.now();

    eventValue == true;

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
    eventValue == true;
  }

  //GPS Loop
  // Gets the GPS coords and tests whether it's in "bounds" - @params: none - @return: void //

  void locationBarrier() {
    while (ss.available() > 0)
      if (gps.encode(ss.read()))
        getGPSInfo();
  }
  
  void getGPSInfo() {
    eventValue == true;
    Serial.print(F("Location: "));
    if (gps.location.isValid()) {
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
      gbootSuccess == true; // since the program has completed SD card and GPS functions, the 'boot' has successfully completeted
      gpsconState = true;
    }
    else {
      Serial.println(F("INVALID"));
      gbootSuccess == false;
    }
  }
