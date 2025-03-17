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

double updateMax(cVector3d position, double voltage,cVector3d& maxPos) {
    static std::deque<cVector3d> lastPositions;
    static std::deque<double> lastIntensities;

    lastPositions.push_back(position);
    lastIntensities.push_back(voltage);

    auto it = std::max_element(lastIntensities.begin(), lastIntensities.end());
    double maxVoltage = *it;  // Get the value
    double maxIndex = std::distance(lastIntensities.begin(), it); // Get the index
    maxPos = lastPositions[maxIndex];
    
    return maxVoltage;
}
