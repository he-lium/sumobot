// IR/Line Sensor variables and functions

namespace ir {
    const int irSensor = 1;
    bool nearBoundary = false;
    void irInterrupt();

    // initialise IR Sensor during setup
    void initIRSensor() {
        pinMode(irSensor, INPUT_PULLUP);
        // Attach interrupt to call when color changes
        attachInterrupt(digitalPinToInterrupt(irSensor), irInterrupt, RISING);
    }

    // Called when IR interrupt occurs (change between black and white)
    void irInterrupt() {
        // digitalRead(irSensor) == 1 when irSensor senses dark
        nearBoundary = (digitalRead(irSensor) == 1);
    }

}