#include "Data_process.h"

// Constructor: Initializes the Kalman filter
KalmanFilter::KalmanFilter(double init_voltage, double process_noise, double measurement_noise):   
        estimated_voltage(init_voltage), 
        estimation_error(1.0), 
        process_noise(process_noise), 
        measurement_noise(measurement_noise) {}

// Apply the Kalman filter to a new voltage measurement
double KalmanFilter::applyFilter(double new_measurement) {
    // Prediction step (no control model, just holding the state constant)

    // Kalman gain calculation
    kalman_gain = estimation_error / (estimation_error + measurement_noise);

    // Update the estimate with the new measurement
    estimated_voltage = estimated_voltage + kalman_gain * (new_measurement - estimated_voltage);

    // Update estimation error
    estimation_error = (1 - kalman_gain) * estimation_error + process_noise;

    return estimated_voltage;
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

struct spacialVoltage
{
    cVector3d RobotPos;
    double signal_value;
   spacialVoltage(cVector3d position, double value) {
        RobotPos = position;
        signal_value = value;
    }
};


cVector3d computeGradient(cVector3d currentRobotPosition,double current_voltage ) {
    static std::deque<spacialVoltage> last_values;
    spacialVoltage tmp(currentRobotPosition, current_voltage);
    if (last_values.size() < 3) {
        last_values.push_back(tmp);
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
        A.row(0) = Eigen::Vector3d( last_values[0].RobotPos.x(), last_values[0].RobotPos.y(), 
                                    last_values[0].RobotPos.z()).transpose();

        A.row(1) = Eigen::Vector3d( last_values[1].RobotPos.x(), last_values[1].RobotPos.y(), 
                                    last_values[1].RobotPos.z()).transpose();

        A.row(2) = Eigen::Vector3d( last_values[2].RobotPos.x(), last_values[2].RobotPos.y(),
                                    last_values[2].RobotPos.z()).transpose();

        // Set up the vector b with the voltage differences
        Eigen::Vector3d b;
        b << V_p1 - V_p0, V_p2 - V_p0, V_p3 - V_p0;

        // Solve the system of equations A * grad = b for the gradient (grad)
        Eigen::Vector3d grad = A.colPivHouseholderQr().solve(b);

        // Convert the gradient to a CHAI3D vector (cVector3d)
        cVector3d gradient(grad[0], grad[1], grad[2]);

        // Add current position and voltage and remove the last one
        last_values.push_back(tmp);
        last_values.pop_front();

        return gradient;
    
    }

}