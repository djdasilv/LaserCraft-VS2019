#include "Data_process.h"

#include <numeric>

GaussianFilter::GaussianFilter(int window_size, double sigma)
    : window_size(window_size), index(0), filled(false) {
    buffer.resize(window_size, 0.0);
    weights.resize(window_size);
    calculateWeights(sigma);
}

void GaussianFilter::calculateWeights(double sigma) {
    int mid = window_size / 2;
    double sum = 0.0;

    for (int i = 0; i < window_size; ++i) {
        int x = i - mid;
        weights[i] = std::exp(-0.5 * (x * x) / (sigma * sigma));
        sum += weights[i];
    }

    // Normalize weights
    for (int i = 0; i < window_size; ++i) {
        weights[i] /= sum;
    }
}

double GaussianFilter::applyFilter(double new_measurement) {
    buffer[index] = new_measurement;
    index = (index + 1) % window_size;

    int count = filled ? window_size : index;
    double filtered_value = 0.0;

    for (int i = 0; i < count; ++i) {
        int weight_index = (i - index + window_size) % window_size;
        filtered_value += buffer[i] * weights[weight_index];
    }

    if (!filled && index == 0) filled = true;

    return filtered_value;
}


double updateMax(cVector3d position, double voltage,cVector3d& maxPos,bool reset) {
    static std::deque<cVector3d> lastPositions;
    static std::deque<double> lastIntensities;

    lastPositions.push_back(position);
    lastIntensities.push_back(voltage);

    auto it = std::max_element(lastIntensities.begin(), lastIntensities.end());
    double maxVoltage = *it;  // Get the value
    double maxIndex = std::distance(lastIntensities.begin(), it); // Get the index
    maxPos = lastPositions[maxIndex];
    cVector3d temporary_max = position;
    if (reset == true) {
        lastIntensities.clear();
        lastPositions.clear();
        maxPos = temporary_max;
        maxVoltage = 0;
        return 0;
    }
    
    return maxVoltage;
}

struct signal3d
{
    cVector3d RobotPos;
    double signal_value;
    signal3d(cVector3d position, double value) {
        RobotPos=position;
        signal_value = value;
    }
};


cVector3d computeGradient(cVector3d currentRobotPosition,double current_voltage ) {
    static std::deque<signal3d> last_values;
    signal3d tmp(currentRobotPosition, current_voltage);
    
    // Check if currentRobotPosition is already in last_values
    bool isValid = true;
    double delta = 0.00001; //m
    double voltagNoiseThreshold = 0.01; //V
    for (const auto& entry : last_values) {
        if (entry.RobotPos.equals(currentRobotPosition) || entry.RobotPos.distance(currentRobotPosition) < delta)
        {
            isValid = false;
            break;
        }
    }
    
    if (last_values.size() < 3) {
        if (isValid)   last_values.push_back(tmp);
        return cVector3d(0, 0, 0);
       
    }
    else {
        // Sample voltages at the three points
        double V_p0 = current_voltage;
        double V_p1 = last_values[0].signal_value;
        double V_p2 = last_values[1].signal_value;
        double V_p3 = last_values[2].signal_value;

        // Set up the matrix A with the displacement vectors
        Eigen::Matrix3d A;
        A.row(0) = Eigen::Vector3d(last_values[0].RobotPos.x() - currentRobotPosition.x(),
            last_values[0].RobotPos.y() - currentRobotPosition.y(),
            last_values[0].RobotPos.z() - currentRobotPosition.z()).transpose();

        A.row(1) = Eigen::Vector3d(last_values[1].RobotPos.x() - currentRobotPosition.x(),
            last_values[1].RobotPos.y() - currentRobotPosition.y(),
            last_values[1].RobotPos.z() - currentRobotPosition.z()).transpose();

        A.row(2) = Eigen::Vector3d(last_values[2].RobotPos.x() - currentRobotPosition.x(),
            last_values[2].RobotPos.y() - currentRobotPosition.y(),
            last_values[2].RobotPos.z() - currentRobotPosition.z()).transpose();

        // Set up the vector b with the voltage differences
        Eigen::Vector3d b;
        b << V_p1 - V_p0, V_p2 - V_p0, V_p3 - V_p0;

        // Solve the system of equations A * grad = b for the gradient (grad)
        Eigen::Vector3d grad = A.colPivHouseholderQr().solve(b);

        // Convert the gradient to a CHAI3D vector (cVector3d)
        cVector3d gradient(grad[0], grad[1], grad[2]);
        gradient.normalize();

        // Add current position and voltage and remove the last one
        if (isValid) {
            last_values.push_back(tmp);
            last_values.pop_front();
        }

        return gradient;
    
    }

}