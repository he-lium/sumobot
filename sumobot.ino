// Robot code for Team 30: MDM
// Sumobot Competition 2017

// LED
const int internalLED = 17;

// Button
const int toggleButton = 10;

// Motor A
const int motorApin1 = 9;
const int motorApin2 = 8; // Reverse direction
const int motorAspeed = 4; // PWM

// Motor B
const int motorBpin1 = 6;
const int motorBpin2 = 7;  // Reverse direction
const int motorBspeed = 5; // PWM

// Ultrasonic 1
const int trigPin = 2;
const int echoPin = 0;
unsigned int us1Duration = 0; // time taken (millis) for ultrasonic to read
unsigned int us1Read = 0; // counter for num of ultrasonic measures

// Ultrasonic 2
const int trigPin2 = 15;
const int echoPin2 = 3;

// Ultrasonic Distance in cm for changing between search and attack
int ultrasonicThreshold = 40;

const int waitingTime = 3000;

enum State { off, waiting, playing };
enum PlayState { search, attack, reverse };
// state machine
State state;
PlayState playState;
unsigned long startPlayTimestamp;
int filter = 0;

void setup() {
    // Set up serial comm
    Serial.begin(9600);

    // Init motors
    pinMode(motorApin1, OUTPUT);
    pinMode(motorApin2, OUTPUT);
    pinMode(motorAspeed, OUTPUT);
    pinMode(motorBpin1, OUTPUT);
    pinMode(motorBpin2, OUTPUT);
    pinMode(motorBspeed, OUTPUT);

    // Set up Ultrasonic
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
    // Ultrasonic 1 Interrupt
    attachInterrupt(digitalPinToInterrupt(echoPin), ultrasonic1_echo, CHANGE);

    // Set up state
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
            runMotors(1, 1, 0, 0);
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
        trigUltrasonic(trigPin);
        ultrasonicTimeSinceLastRead = millis();
    }

    switch(playState) {
    case search:
        runMotors(1, 0, 140, 140);
        if (currentUs1Read != us1Read) {
            // new update from us1
            currentUs1Read = us1Read;
            // calculate distance from ultrasonic
            int distance = us1Duration / 58;
            Serial.print("search: ");
            Serial.println(distance);

            if (distance < ultrasonicThreshold) {
                filter++;
                if (filter >= 10) { // if distance is consistently above threshold
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
        runMotors(1, 1, 140, 140);
        if (currentUs1Read != us1Read) {
            // new update from us1
            currentUs1Read = us1Read;
            // calculate distance from ultrasonic
            int distance = us1Duration / 58;
            Serial.print("attack: ");
            Serial.println(distance);
            
            if (distance > ultrasonicThreshold) {
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
        // TODO go back for period of time
        break;
    }
}

bool readToggleButton() {
    return digitalRead(toggleButton) == HIGH;
}

// Set the motors to a particular direction and speed
// directions: 1 forward, 0 backward
// speed: 0
void runMotors(int directionA, int directionB, int speedA, int speedB)
{
    // Motor A
    if (directionA == 1) { // forward
        digitalWrite(motorApin1, HIGH);
        digitalWrite(motorApin2, LOW);
    } else { // backward
        digitalWrite(motorApin1, LOW);
        digitalWrite(motorApin2, HIGH);
    }
    analogWrite(motorAspeed, speedA);

    // Motor B
    if (directionB == 1) { // forward
        digitalWrite(motorBpin1, HIGH);
        digitalWrite(motorBpin2, LOW);
    } else { // backward
        digitalWrite(motorBpin1, LOW);
        digitalWrite(motorBpin2, HIGH);
    } 
    analogWrite(motorBspeed, speedB);
}

// ********************************************
// Ultrasonic Sensor:

// WARNING: WILL BLOCK
// reads distance of ultrasonic
int measureUltrasonic() {
    // sensor is triggered by HIGH pulse of 10 or more seconds
    // give short LOW pulse beforehand
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // read sig from sensor, convert into dist
    return pulseIn(echoPin, HIGH) / 58;
}

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
    switch (digitalRead(echoPin)) {
    case HIGH: // start of echo pulse
        startTime = micros();
        break;
    case LOW: // end of echo pulse; record duration
        us1Duration = micros() - startTime;
        us1Read++;
        break;
    }
}
