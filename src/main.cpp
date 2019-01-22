#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <Adafruit_NeoPixel.h>
#include <QueueArray.h>
#include <Filter.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 2
#define USE_SERIAL Serial

/***************************** ULTRASONIC SENSOR SETUP *****************************/
const int TRIG_PIN = 13;
const int ECHO_PIN = 16;
const float CAL_FACTOR = 0.0344; //Calibration factor, this value is based on sound velocity

const int WEIGHT = 5; // used in exponential filter to select the precision level, higher is better
ExponentialFilter<int> UltrasonicFilter(WEIGHT,0);  

int lastSensorDistance = 0; // used to track the last distance measured from the sensor
int sameValueInRow = 0; // tracks the number of same distance values that were recorderd in a row
const int HOW_MANY_IN_ROW = 1000; // tells how many same values should be recorded to declare warning state

unsigned long checkSensorTime = millis(); // time interval at which we sense the movement
const int CHECK_SENSOR_INTERVAL = 100;

unsigned long sensorReadingTime = millis(); // make the sensor read at a certain time interval
const int SENSOR_READING_TIME = 10;

const int DISTANCE_THRESHOLD = 4; // how much a given distance measurement can deviate

bool movementDetected = false;

ESP8266WiFiMulti WiFiMulti;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

// 4467cbf45ae3
const char* ssid     = "NOS-ED00";
const char* password = "4467cbf45ae3";

const int tableNumber = 11;

// get this from the wia dashboard. it should start with d_sk
const char* device_secret_key = "d_sk_OKFBxly8PVZhh74NlkgOIYB8";

const int redLedPin= 15;
const int greenLedPin = 13;
const int blueLedPin = 0;

const int startPushButton = 14;
const int endPushButton = 12;

const int freeState = 0;
const int occupiedState = 1;
const int warningState = 2;

int tableState = 0;
unsigned long timer = millis();
unsigned long postTimer = millis();

int val = 0;
int val2 = 0;

int webSiteState = freeState;

QueueArray<int> postsQueue;

long convertToMinutes(long milli);
void lightUpLed(int state);
void checkOccupiedState();
void checkWarningState();
void pressedLeaveButton();
void pressedOccupiedButton();
void test();
void sendPostsInQueue();
void sendPost(int state);
void sendPostToServer(int state);
void colorWipe(uint32_t c, uint8_t wait);
void senseMovement();
void getDistanceFromSensor();
void leavePressed();
void occupiedPressed();

bool pressedLeave = false;
bool pressedOccupied = false;

void setup() {
  USE_SERIAL.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP(ssid, password);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);

  pinMode(startPushButton, INPUT_PULLUP);
  pinMode(endPushButton, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(endPushButton),leavePressed, RISING);
  attachInterrupt(digitalPinToInterrupt(startPushButton),occupiedPressed, RISING);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(30);
  colorWipe(strip.Color(0, 255, 0), 50); // Green
  getDistanceFromSensor();
  lastSensorDistance = UltrasonicFilter.Current();
}

void leavePressed() {
  pressedLeave = true;
}

void occupiedPressed() {
  pressedOccupied = true;
}

void pressedLeaveButton() {
  tableState = freeState;
  lightUpLed(freeState);
  sendPost(freeState);
}

void pressedOccupiedButton() {
  tableState = occupiedState;
  lightUpLed(occupiedState);
  timer = millis();
  sameValueInRow = 0;
  sendPost(occupiedState);  
}

void loop() {
  if (pressedOccupied) {
    pressedOccupiedButton();
    pressedOccupied = false;
  }
  if (pressedLeave) {
    pressedLeaveButton();
    pressedLeave = false;
  }

  if (movementDetected) {
    pressedOccupiedButton();
    movementDetected = false;
  }
  if(tableState == occupiedState) {
      checkOccupiedState();
  } else if(tableState == warningState) {
      checkWarningState();  
  }
  sendPostsInQueue();
  getDistanceFromSensor();
  senseMovement();
}

long convertToMinutes(long milli) {
  return milli / 60000;  
}

void sendPostsInQueue() {
  unsigned long timeDifference = millis() - postTimer;
  if (timeDifference >= 100) {
    if (postsQueue.count() > 0) {
      sendPostToServer(postsQueue.pop());
    }
    postTimer = millis();
  }
}

void sendPost(int state) {
  postsQueue.push(state);
}

void sendPostToServer(int state) {
    if((WiFiMulti.run() == WL_CONNECTED)) {
        HTTPClient http;
        String postData = "lugar=" + String(tableNumber) + "&status=" + String(state);

        http.begin("http://web.tecnico.ulisboa.pt/ist182015/add.php");
        USE_SERIAL.print("[HTTP] POST...\n");

        http.addHeader("Content-Type", "application/x-www-form-urlencoded");

        // start connection and send HTTP headers. replace name and data values with your own.
        int httpCode = http.POST(postData);

        
       
        http.end();
        webSiteState = state;
    } else {
      USE_SERIAL.println("NOT CONNECTED!");
    }
}
void checkOccupiedState() {
  long timeDifference = millis() - timer;
  
  /*if(convertToMinutes(timeDifference) >= 1) {
    tableState = warningState;
    lightUpLed(warningState);
    timer = millis(); 
  }*/
  if(timeDifference >= 10000) {
    tableState = warningState;
    lightUpLed(warningState);
    timer = millis(); 
  }
}

void checkWarningState() {
  long timeDifference = millis() - timer;
  
  /*if(convertToMinutes(timeDifference) >= 1) {
    tableState = freeState;
    lightUpLed(freeState);
    sendPost(freeState);
  }*/
  if(timeDifference >= 10000) {
    tableState = freeState;
    lightUpLed(freeState);
    sendPost(freeState);
  }
}

void lightUpLed(int state) {
  if(state == freeState) {
    colorWipe(strip.Color(0, 255, 0), 50); // Green
  } else if(state == occupiedState) {
    colorWipe(strip.Color(255, 0, 0), 50); // Red  
  } else if(state == warningState) {
    colorWipe(strip.Color(230, 184, 0), 50); // Yellow
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

// Function to ping the ultrasonic sensor
void senseMovement() {
  unsigned long timeDifference = millis() - checkSensorTime;
  if (timeDifference >= CHECK_SENSOR_INTERVAL) {
    int currentSensorDistance = UltrasonicFilter.Current();
    if ((currentSensorDistance >= lastSensorDistance + DISTANCE_THRESHOLD) || (currentSensorDistance >= lastSensorDistance - DISTANCE_THRESHOLD)) {
      sameValueInRow++;
    } else {
      sameValueInRow = 0;
      //pressedOccupiedButton();
      movementDetected = true;
    }
    if (sameValueInRow == HOW_MANY_IN_ROW) { 
      sameValueInRow = 0;
      if (tableState == occupiedState) {
        tableState = warningState;
      }
    }
    lastSensorDistance = currentSensorDistance;
    checkSensorTime = millis();
  }
  USE_SERIAL.println("filter: " + String(UltrasonicFilter.Current()) + " cm");
}


void getDistanceFromSensor() {
  unsigned long timeDifference = millis() - sensorReadingTime;
  if (timeDifference >= SENSOR_READING_TIME) {
    float duration;
    int distance;
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = (duration / 2) * CAL_FACTOR;
    UltrasonicFilter.Filter(distance);
    sensorReadingTime = millis();
  }
}