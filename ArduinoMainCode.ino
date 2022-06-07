//SD Card Global
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"
RTC_Millis rtc;     // Software Real Time Clock (RTC)
DateTime rightNow;  // used to store the current time.


//GPS Global
#include <SoftwareSerial.h>
#include <TinyGPS.h>
float lat = 28.5458, lon = 77.1703; // create variable for latitude and longitude object
SoftwareSerial gpsSerial(17, 16); //rx,tx
TinyGPS gps; // create gps object



//Servo Global
#include <Servo.h>
Servo servo;


//DC Global
#include <L298N.h>
// Pin definition
const unsigned int IN3 = 22;
const unsigned int IN4 = 23;
// Create one motor instance
L298N motor(IN3, IN4);

//Sonar Global
int distance;

//Button 1 Global
int button1State = 0;
int inPin = 8;
int valueCrash = 1;

//Button 2 Global
int button2State = 0;

//Button 3 Global
int button3State = 0;



void setup() {
  Serial.begin(9600);           // Open serial communications and wait for port to open:
  while (!Serial) {
    delay(1);                   // wait for serial port to connect. Needed for native USB port only
  }


  // SD Card initialisation
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  // Real Time Clock (RTC)
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println("initialization done.");



    //GPS Setup
    gpsSerial.begin(9600); // connect gps sensor;

  
      //Lights Setup
      pinMode(A1, OUTPUT);
      pinMode(A2, OUTPUT);
      pinMode(A3, OUTPUT);


        //Piezo Setup
        pinMode(48, OUTPUT);


          //Servo Setup
          servo.attach(6);


            //DC Setup
            motor.setSpeed(70);
            motor.forward();


              //Sonar Setup
              pinMode(3, OUTPUT); // Sets the trigPin as an OUTPUT
              pinMode(4, INPUT); // Sets the echoPin as an INPUT


                //Line Setup
                pinMode(7, INPUT);


                  //Button 1 Setup
                  pinMode(A1, OUTPUT);
                  pinMode(inPin, INPUT_PULLUP);


                    //Button 2 Setup
                    pinMode(9, INPUT);


                      //Button 3 Setup
                      pinMode(10, INPUT);
}




void loop() {
  int logEvent;
  int GPSLoop;
  int lightsLoop;
  int piezoLoop;
  int servoLoop;
  int sonarLoop;
  int lineLoop;
  int potLoop;
  int button1Loop;
  int button2Loop;
  int button3Loop;
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



// GPS Loop
void GPSLoop() {
  while (gpsSerial.available()) { // check for gps data
    if (gps.encode(gpsSerial.read())) // encode gps data
    {
      gps.f_get_position(&lat, &lon); // get latitude and longitude
      // Serial.print("Position: ");
      // Serial.print("Latitude:");
      // Serial.print(lat,6);
      // Serial.print(";");
      // Serial.print("Longitude:");
      // Serial.println(lon,6);
      // Serial.print(lat);
      // Serial.print(" ");
    }
  }

  String latitude = String(lat, 6);
  String longitude = String(lon, 6);
  Serial.println(latitude + ";" + longitude);
  delay(1000);
}



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



void piezoLoop() {
  tone(48, 1000, 500);
}



void servoLoop() {
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
}



void DCLoop() {
  //
}



void sonarLoop() {
long duration;

  // Clears the trigPin condition
  digitalWrite(3, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(3, HIGH);
  delayMicroseconds(10);
  digitalWrite(3, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(4, HIGH);

  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  delay(300);
  Serial.print(distance);
  delay(300);
  Serial.println(" cm");
}



void lineLoop() {
  bool value = digitalRead(7);
  if (value == 0) {
    Serial.println("Yay");
  }
}



void potLoop() {
  int sensorValue = analogRead(A0); //read the input on analog pin 0
  Serial.println(sensorValue); //print out the value you read
  delay(300); //delay in between reads for stability
}



void button1Loop() {
  valueCrash = digitalRead(inPin); // read the state of the crash sensor
  if (valueCrash == 0) {
    digitalWrite(A1, HIGH);
  } else {
    digitalWrite(A1, LOW);
  }
}


void button2Loop() {
  // read the state of the pushbutton value:
  int buttonState = digitalRead(9);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == 0) {
    // turn LED on:
    Serial.println("LED_BUILTIN, HIGH");
  } else {
    // turn LED off:
    Serial.println("LED_BUILTIN, LOW");
  }
}


void button3Loop() {
  // read the state of the pushbutton value:
  int buttonState = digitalRead(9);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == 0) {
    // turn LED on:
    Serial.println("LED_BUILTIN, HIGH");
  } else {
    // turn LED off:
    Serial.println("LED_BUILTIN, LOW");
  }
}
