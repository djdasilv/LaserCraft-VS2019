#pragma once

#include <vector>
#include <deque>
#include "chai3d.h"
#include "cbw.h"
#include <GLFW/glfw3.h>

using namespace chai3d;
using namespace std;


class KalmanFilter {
public:
    // Kalman Filter variables
    double estimated_voltage;    // Estimated voltage (state)
    double estimation_error;     // Estimation error (uncertainty in the state)
    double process_noise;        // Process noise covariance (model uncertainty)
    double measurement_noise;    // Measurement noise covariance (sensor uncertainty)
    double kalman_gain;          // Kalman Gain

    // Constructor: Initializes the Kalman filter
    KalmanFilter(double init_voltage, double process_noise, double measurement_noise);

    // Apply the Kalman filter to a new voltage measurement
    double applyFilter(double new_measurement);
};

double updateMax(cVector3d position,double voltage,cVector3d& maxPos, bool reset) ;

