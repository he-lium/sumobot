// Class definition for filter
#ifndef FILTER_H
#define FILTER_H

class Filter {
public:
    Filter(int numDim);
    ~Filter();
    void registerFar(int dim); // dim range (1..numDim)
    void registerNear(int dim);
    void reset();
    bool isNear(int dim) const;
    const int filterThreshold = 10;
private:
    int _numDim;
    // Array of (whether ultrasonic detects near)
    bool *_isNear;
    // Array of filter int
    int *_closeness;
};

Filter::Filter(int numDim) {
    _numDim = numDim;
    _isNear = new bool[numDim]();
    _closeness = new int[numDim]();
}

// Register ultrasonic's FAR reading to filter
void Filter::registerFar(int numDim) {
    int d = numDim - 1;
    if (_closeness[d] > 0) {
        --_closeness[d];
        // Serial.print(" ");
        // Serial.print(_closeness[d]);
        if (_closeness[d] == 0) _isNear[d] = false;
    }
}

// Register ultrasonic's NEAR reading to filter
void Filter::registerNear(int numDim) {
    int d = numDim - 1;
    if (_closeness[d] < filterThreshold) {
        ++_closeness[d];
        // Serial.print(" ");
        // Serial.print(_closeness[d]);
        if (_closeness[d] == filterThreshold) _isNear[d] = true;
    }
}

bool Filter::isNear(int numDim) const { return _isNear[numDim - 1]; }

void Filter::reset() {
    for (int i = 0; i < _numDim; ++i) {
        _isNear[i] = false;
        _closeness[i] = 0;
    }
}

Filter::~Filter() {
    delete [] _isNear;
    delete [] _closeness;
}

#endif
