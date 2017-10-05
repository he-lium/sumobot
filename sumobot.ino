// Robot code for Team 30: MDM
// Sumobot Competition 2017

#include "motor.h"
#include "ir.h"
#include "filter.h"

// LED ouput
const int internalLED = 17;

// Push button input
const int toggleButton = 10;

// Ultrasonic 1
const int us1trigPin = 2; // Trigger output
const int us1echoPin = 0; // Echo input interrupt
unsigned int us1Distance = 0; // distance reading from us1
unsigned int us1Read = 0; // counter for num of ultrasonic measures

// Ultrasonic 2
const int us2trigPin = 18;
const int us2echoPin = 3;
unsigned int us2Distance = 0; // distance reading from us2
unsigned int us2Read = 0; // counter for num of ultrasonic measures

// Ultrasonic Distance in cm for changing between search and attack
const int ultrasonicThreshold = 40;

// Maximum Valid Ultrasonic Distance
const int maxValidUltrasonic = 900;

// Time Duration (in millis) of reverse
const int maxReverseTime = 1500;

// Time in millis to wait after button press before playing
const int waitingTime = 3000;

// state machine
enum State { off, waiting, playing };
State state;
enum PlayState { search, attack, reverse };
PlayState playState;
// Timestamp to record when buttons were pressed
unsigned long startPlayTimestamp;

// Timestamp at start of reverse
unsigned long reverseTimestamp;

// Filter object for ultrasonic consistency
Filter *usFilter;

void setup() {
    // Set up serial comm
    Serial.begin(9600);

    ir::initIRSensor();
    motor::initMotors();

    // Set up Ultrasonic 1
    pinMode(us1trigPin, OUTPUT);
    pinMode(us1echoPin, INPUT);
    digitalWrite(us1trigPin, LOW);
    // Ultrasonic 1 Interrupt : call ultrasonic1_echo when echo pin changes
    attachInterrupt(digitalPinToInterrupt(us1echoPin), ultrasonic1_echo, CHANGE);

    // Set up Ultrasonic 2
    pinMode(us2trigPin, OUTPUT);
    pinMode(us2echoPin, INPUT);
    digitalWrite(us2trigPin, LOW);
    // Ultrasonic 2 Interrupt TODO: FIX
    attachInterrupt(digitalPinToInterrupt(us2echoPin), ultrasonic2_echo, CHANGE);

    // Set up initial state: waiting for button press
    state = off;

    // Set up filter
    usFilter = new Filter(2);

    delay(500);
    Serial.println("Sumobot MDM v0.1");
}

void loop() {
    switch (state) {
    case off:
        // Waiting for button press
        if (readToggleButton()) {
            state = waiting;
            startPlayTimestamp = millis();
            digitalWrite(internalLED, HIGH);
            Serial.println(digitalPinToInterrupt(us2echoPin));
            Serial.println("Waiting...");
        }
        
        break;
    case waiting:
        // Wait 3 seconds
        if (millis() - startPlayTimestamp >= waitingTime) {
            state = playing;
            playState = search;
            digitalWrite(internalLED, LOW);
            usFilter->reset();
            Serial.println("Playing!");
        }
        break;
    case playing:

        // Turn off if button pressed
        if (readToggleButton()) {
            motor::runMotors(1, 1, 0, 0);
            state = off;
            Serial.println("Stopping game...");
            delay(1000);
            break;
        }

        decidePlay();
        break; // from outer switch   
    }
}

void printUs() {
    Serial.print("Ultrasonic 1: ");
    Serial.println(us1Distance);
    Serial.print("Ultrasonic 2: ");
    Serial.println(us2Distance);
}

// decide what to do when playing
void decidePlay() {
    // ultrasonic update counter
    static unsigned long currentUs1Read = 0;
    // time (millis) since last ultrasonic read
    static unsigned long ultrasonicTimeSinceLastRead = 0;

    // ultrasonic trigger
    if (millis() - ultrasonicTimeSinceLastRead > 7) {
        // start new ultrasonic read
        trigUltrasonic(us1trigPin);
        trigUltrasonic(us2trigPin);
        ultrasonicTimeSinceLastRead = millis();
    }

    // receiving ultrasonic results
    if (currentUs1Read != us1Read) { // us1 has changed
        if (us1Distance <= ultrasonicThreshold) // NEAR
            usFilter->registerNear(1);
        else // FAR
            usFilter->registerFar(1);
        currentUs1Read = us1Read;
    }

    // near boundary: reverse
    if (ir::nearBoundary && playState != reverse) {
        Serial.println("Near boundary; reverse");
        playState = reverse;
        reverseTimestamp = millis();
    }

    switch(playState) {
    case search:
        motor::runMotors(1, 0, 140, 140);
        if (usFilter->isNear(1)) {
            playState = attack;
            Serial.println("Distance below threshold; attack");
        }
        break;
        case attack:
        motor::runMotors(1, 1, 140, 140);
        if (!usFilter->isNear(1)) { // far away
            playState = search;
            Serial.println("Distance above threshold; search");
        }
        break;
    case reverse:
        // go back for period of time
        motor::runMotors(1, 1, 140, 140);
        
        // if robot has reversed for enough time
        if (millis() - reverseTimestamp >= maxReverseTime) {
            Serial.println("Stop reversing; search");
            playState = search;
        }
        
        break;
    }
}

////////////////////////////////////////////
//// PUSH BUTTON

bool readToggleButton() {
    return digitalRead(toggleButton) == HIGH;
}

// ********************************************
// Ultrasonic Sensor:

// Sends trigger to activate ultrasonic
int trigUltrasonic(int trigger) {
    // sensor is triggered by HIGH pulse of 10 or more seconds
    // give short LOW pulse beforehand
    digitalWrite(trigger, LOW);
    delayMicroseconds(5);
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);
}

// interrupt call when ultrasonic1 starts echo
void ultrasonic1_echo() {
    static volatile unsigned long startTime;
    // Serial.println("TRIGGER 1");
    switch (digitalRead(us1echoPin)) {
    case HIGH: // start of echo pulse
        startTime = micros();
        break;
    case LOW: // end of echo pulse; record duration
        us1Distance = (micros() - startTime) / 58;
        // Only mark as updated if valid distance
        if (us1Distance < maxValidUltrasonic) us1Read++;
        break;
    }
}

// interrupt call when ultrasonic2 starts echo
void ultrasonic2_echo() {
    static volatile unsigned long startTime2;
    // Serial.println("TRIGGER 2");
    switch (digitalRead(us2echoPin)) {
    case HIGH: // start of echo pulse
        startTime2 = micros();
        break;
    case LOW: // end of echo pulse; record duration
        us2Distance = (micros() - startTime2) / 58;
        if (us2Distance < maxValidUltrasonic) us2Read++;
        break;
    }
}
