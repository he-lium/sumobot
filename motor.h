// Operation of output motors

#ifndef MOTOR_H
#define MOTOR_H

namespace motor {
    const int defaultSpeed = 150;

    // Motor A: right motor
    const int motorApin1 = 9;
    const int motorApin2 = 8; // Reverse direction
    const int motorAspeed = 10; // PWM ENA

    // Motor B: left motor
    const int motorBpin1 = 14;
    const int motorBpin2 = 6;  // Reverse direction
    const int motorBspeed = 5; // PWM ENB

    // initialise motors during setup
    void initMotors() {
        pinMode(motorApin1, OUTPUT);
        pinMode(motorApin2, OUTPUT);
        pinMode(motorAspeed, OUTPUT);
        pinMode(motorBpin1, OUTPUT);
        pinMode(motorBpin2, OUTPUT);
        pinMode(motorBspeed, OUTPUT);
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
        speedB *= 1.16;
        if (directionB == 1) { // forward
            digitalWrite(motorBpin1, HIGH);
            digitalWrite(motorBpin2, LOW);
        } else { // backward
            digitalWrite(motorBpin1, LOW);
            digitalWrite(motorBpin2, HIGH);
        } 
        analogWrite(motorBspeed, speedB);
    }

    // Running motor abstractions
    void forwards() { runMotors(1, 1, defaultSpeed * 1.4, defaultSpeed * 1.4); }
    void reverse() { runMotors(0, 0, defaultSpeed, defaultSpeed); }
    void anticlockwise() { runMotors(1, 0, defaultSpeed, defaultSpeed); }
    void clockwise() { runMotors(0, 1, defaultSpeed, defaultSpeed); }
    void veerleft() { runMotors(1, 1, defaultSpeed * 1.3, defaultSpeed * 0.9); }
    void veerright() { runMotors(1, 1, defaultSpeed * 0.8, defaultSpeed * 1.3); }
}

#endif