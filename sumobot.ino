// Robot code for Team 30: MDM
// Sumobot Competition 2017

#include "motor.h"
#include "ir.h"

// LED ouput
const int internalLED = 17;

// Push button input
const int toggleButton = 10;

// Ultrasonic 1
const int us1trigPin = 2; // Trigger output
const int us1echoPin = 0; // Echo input interrupt
unsigned int us1Duration = 0; // time taken (millis) for ultrasonic to read
unsigned int us1Read = 0; // counter for num of ultrasonic measures

// Ultrasonic 2
const int us2trigPin = 15;
const int us2echoPin = 3;
unsigned int us2Duration = 0; // time taken (millis) for ultrasonic to read
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
// Filter to ensure consistenty of ultrasonic values
int filter = 0;
// Timestamp at start of reverse
unsigned long reverseTimestamp;

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
    // TODO Ultrasonic 2 Interrupt
    attachInterrupt(digitalPinToInterrupt(us2echoPin), ultrasonic2_echo, CHANGE);

    // Set up initial state: waiting for button press
    state = off;

    delay(500);
}

void loop() {
    switch (state) {
    case off:
        // Waiting for button press
        if (readToggleButton()) {
            state = waiting;
            startPlayTimestamp = millis();
            digitalWrite(internalLED, HIGH);
            Serial.println("Waiting...");
        }
        break;
    case waiting:
        // Wait 3 seconds
        if (millis() - startPlayTimestamp >= waitingTime) {
            state = playing;
            playState = search;
            digitalWrite(internalLED, LOW);
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
        ultrasonicTimeSinceLastRead = millis();
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
        if (currentUs1Read != us1Read) {
            // new update from us1
            currentUs1Read = us1Read;
            // calculate distance from ultrasonic
            int us1distance = us1Duration / 58;
            // Serial.print("search ");
            // Serial.println(us1distance);

            if (us1distance < ultrasonicThreshold) {
                filter++;
                if (filter >= 10) { // if us1distance is consistently above threshold
                    // Change state: attack
                    filter = 0;
                    playState = attack;
                    Serial.println("Distance below threshold; attack");
                }
            } else if (filter > 0) {
                filter--;
            }
        }
        break;
    case attack:
        motor::runMotors(1, 1, 140, 140);
        if (currentUs1Read != us1Read) {
            // new update from us1
            currentUs1Read = us1Read;
            // calculate distance from ultrasonic
            int us1distance = us1Duration / 58;
            Serial.print("");
            // Serial.print("attack ");
            // Serial.println(us1distance);
            
            if (us1distance > ultrasonicThreshold) {
                // Distance starting to be greater than threshold
                filter++;
                if (filter >= 10) { // if distance is consistently above threshold
                    // Change state: search
                    filter = 0;
                    playState = search;
                    Serial.println("Distance above threshold; search");
                }
            } else if (filter > 0) {
                filter--;
            }
        }
        break;
    case reverse:
        // go back for period of time
        motor::runMotors(1, 1, 140, 140);
        
        // if robot has reversed for enough time
        if (millis() - reverseTimestamp >= maxReverseTime) {
            Serial.println("Stop reversing; search");
            filter = 0;
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
    static volatile unsigned int startTime;
    switch (digitalRead(us1echoPin)) {
    case HIGH: // start of echo pulse
        startTime = micros();
        break;
    case LOW: // end of echo pulse; record duration
        us1Duration = micros() - startTime;
        // Only mark as updated if valid distance
        if (us1Duration < maxValidUltrasonic * 58) us1Read++;
        break;
    }
}

// interrupt call when ultrasonic2 starts echo
void ultrasonic2_echo() {
    static volatile unsigned int startTime;
    Serial.println("TRIGGER TRIGGER");
    switch (digitalRead(us2echoPin)) {
    case HIGH: // start of echo pulse
        startTime = micros();
        break;
    case LOW: // end of echo pulse; record duration
        us2Duration = micros() - startTime;
        if (us2Duration < maxValidUltrasonic * 58) {
            us2Read++;
            Serial.println(us2Duration / 58);
        }
    }
}