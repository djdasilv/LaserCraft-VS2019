#pragma once

#include <vector>
#include <deque>


class GaussianFilter {
private:
    std::deque<double> window;   // Circular buffer storing recent measurements
    std::vector<double> kernel;  // Precomputed Gaussian kernel
    int window_size;
    double sigma;

public:
    // Constructor with default value for window size and sigma
    GaussianFilter(int size=5, double sigma=2);

    // Method to apply the Gaussian filter
    double applyFilter(double new_measurement);
};

