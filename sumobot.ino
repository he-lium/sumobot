// Robot code for Team 30: MDM
// Sumobot Competition 2017

#include "motor.h"
#include "ir.h"
#include "filter.h"

// LED ouput
const int internalLED = 17;

// Push button input
const int toggleButton = 20;

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

// Used for alternating triggers
bool alternateUSTrigger = false;
bool usingUltrasonic = false; // whether or not ultrasonic is waiting for echo

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
enum PlayState { search, attack, reverse, detectedL, detectedR, reverseRotate };
PlayState playState;
// Timestamp to record when buttons were pressed
unsigned long startPlayTimestamp;
// Timestamp to record when rotating after reversing
unsigned long revRotTimestamp;
bool revRotDirection;
const int revRotDuration = 800;

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

// #define DEBUG
#ifdef DEBUG

void loop() {
    // calibrate motors
    motor::forwards();
    delay(5000);
    motor::runMotors(1, 1, 0, 0);
    delay(3000);
}

#else

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
        // TODO REMOVE DEBUG
        if (millis() % 1000 == 0) printUs();

        decidePlay();
        break; // from outer switch   
    }
}

#endif // #ifdef DEBUG

void printUs() {
    Serial.print("Ultrasonic 1: ");
    Serial.println(us1Distance);
    Serial.print("Ultrasonic 2: ");
    Serial.println(us2Distance);
}

void changeState(PlayState s) {
    if (playState != s) {
        // state is changed; print state
        playState = s;
        digitalWrite(internalLED, LOW);
        switch (playState) {
        case search: Serial.println("Search"); break;
        case attack: Serial.println("Attack"); digitalWrite(internalLED, HIGH); break;
        case reverse: Serial.println("Reverse"); break;
        case detectedL: Serial.println("Detected on left"); break;
        case detectedR: Serial.println("Detected on right"); break;
        case reverseRotate: 
            Serial.println("RotateReverse");
            revRotTimestamp = millis();
            revRotDirection = millis() % 2 == 0;
            break;
        }
    }
}


// ultrasonic update counter
unsigned long currentUs1Read = 0, currentUs2Read = 0;
// time (millis) since last ultrasonic read
unsigned long ultrasonicTimeSinceLastRead = 0;

// decide what to do when playing
void decidePlay() {

    // ultrasonic trigger
    if (millis() - ultrasonicTimeSinceLastRead > 21) {
        // Time taken is too long; override
        usingUltrasonic = false;
        alternateUSTrigger = !alternateUSTrigger;
    }
    if (millis() - ultrasonicTimeSinceLastRead > 7 && !usingUltrasonic) {
        // start new ultrasonic read
        trigUltrasonic(alternateUSTrigger ? us1trigPin : us2trigPin);
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
    if (currentUs2Read != us2Read) {
        if (us2Distance <= ultrasonicThreshold) // NEAR
            usFilter->registerNear(2);
        else
            usFilter->registerFar(2);
        currentUs2Read = us2Read;
    }

    // near boundary: reverse
    if (ir::nearBoundary && playState != reverse) {
        Serial.println("Near boundary; reverse");
        changeState(reverse);
        reverseTimestamp = millis();
    } else if (playState == reverse && !ir::nearBoundary) {
        // out of the edge zone; rotate
        changeState(reverseRotate);
    }
    
    // Decide motor action
    switch(playState) {
    case detectedL:
        motor::veerleft();
        break;
    case detectedR:
    case search:
        motor::veerright();
        break;
    case attack:
        Serial.print("");
        motor::forwards();
        break;
    case reverse:
        // go back for period of time
        motor::reverse();
        break;
    case reverseRotate:
        if (revRotDirection) motor::anticlockwise();
        else motor::clockwise();
        if (millis() - revRotTimestamp > revRotDuration) {
            changeState(search);
        }
        break;
    }

    // Decide new state
    if (playState != reverse && playState != reverseRotate) {
        bool leftNear = usFilter->isNear(1);
        bool rightNear = usFilter->isNear(2);
        if (leftNear && rightNear) changeState(attack);
        else if (leftNear) changeState(detectedL);
        else if (rightNear) changeState(detectedR);
        else changeState(search);
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
        usingUltrasonic = true;
        break;
    case LOW: // end of echo pulse; record duration
        us1Distance = (micros() - startTime) / 58;
        usingUltrasonic = false;
        // Only mark as updated if valid distance
        if (us1Distance < maxValidUltrasonic) us1Read++;
        alternateUSTrigger = !alternateUSTrigger;
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
        usingUltrasonic = true;
        break;
    case LOW: // end of echo pulse; record duration
        us2Distance = (micros() - startTime2) / 58;
        usingUltrasonic = false;
        alternateUSTrigger = !alternateUSTrigger;
        if (us2Distance < maxValidUltrasonic) us2Read++;
        break;
    }
}
