#include "Data_process.h"

GaussianFilter::GaussianFilter(int size, double sigma) : window_size(size), sigma(sigma) {
    kernel.resize(window_size);
    double sum = 0.0;
    double half_size = window_size / 2;

    for (double i = 0; i < window_size; ++i) {
        double x = i - half_size;
        kernel[i] = exp(-0.5 * (x * x) / (sigma * sigma));
        sum += kernel[i];
    }

    // Normalize the kernel to ensure weights sum to 1
    for (double& val : kernel) {
        val /= sum;
    }
}

double GaussianFilter::applyFilter(double new_measurement) {
    if (window.size() >= window_size) {
        window.pop_front(); // Remove oldest value
    }
    window.push_back(new_measurement);

    // Apply Gaussian filter
    double filtered_value = 0.0;
    int kernel_start = window_size - window.size(); // Align kernel for partial windows

    for (size_t i = 0; i < window.size(); ++i) {
        filtered_value += window[i] * kernel[kernel_start + i];
    }

    return filtered_value;
}

bool find_focus(cVector3d position, double voltage, cVector3d& maxPos){
    static std::deque<cVector3d> lastPositions;
    static std::deque<double> lastIntensities;

    if (lastPositions.size() >= 100) {
        lastPositions.pop_front();
        lastPositions.push_back(position);

        lastIntensities.pop_front();
        lastIntensities.push_back(voltage);
    }
    else {
        lastPositions.push_back(position);
        lastIntensities.push_back(voltage);
    }

    if (lastPositions.size() < 100) return false;

    bool decreasing = true;
    for (size_t i = lastIntensities.size() - 50; i < lastIntensities.size() - 1; ++i) {
        if (lastIntensities[i] <= lastIntensities[i + 1]) {
            decreasing = false;
            break; // No need to check further
        }
    }
    int maxIndex = 0;
    double maxVoltage = lastIntensities[0];
    if (decreasing) {
        double maxVoltage = lastIntensities[0];
        for (size_t i = 1; i < lastIntensities.size(); i++) {
            if (lastIntensities[i] > maxVoltage) {
                maxVoltage = lastIntensities[i];
                maxIndex = i;
            }
        }
        maxPos = lastPositions[maxIndex];
        return true; // Focus found
    }
    return false; // No focus detected
}