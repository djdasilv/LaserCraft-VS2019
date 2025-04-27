#pragma once

#include <vector>
#include <deque>
#include "chai3d.h"
#include "cbw.h"
#include <GLFW/glfw3.h>

using namespace chai3d;
using namespace std;


class GaussianFilter {
public:
    /**
     * Constructor
     * @param window_size: Number of past measurements to consider
     * @param sigma: Standard deviation for Gaussian weights
     */
    GaussianFilter(int window_size, double sigma);

    /**
     * Apply the Gaussian filter to a new measurement
     * @param new_measurement: The new input value to be filtered
     * @return: The smoothed value
     */
    double applyFilter(double new_measurement);

private:
    std::vector<double> buffer;     // Circular buffer to store measurements
    std::vector<double> weights;    // Gaussian weights
    int window_size;                // Size of the filter window
    int index;                      // Current insert index
    bool filled;                    // Whether buffer has been fully filled at least once

    /**
     * Precomputes Gaussian weights based on sigma
     */
    void calculateWeights(double sigma);
};

double updateMax(cVector3d position,double voltage,cVector3d& maxPos, bool reset) ;

cVector3d computeGradient(cVector3d currentRobotPosition, double current_voltage);

cVector3d computeSignalDif(cVector3d currentRobotPosition, double current_voltage);

