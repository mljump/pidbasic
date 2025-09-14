//PID draft

#include "vex.h"
using namespace vex;

// ----- CONFIGURATION SETTINGS ----- //
const double kWheelDiameter = /* Enter wheel diameter (inches or cm) */;
const int kNumWheels = /* Enter number of wheels used for drive, e.g., 2, 4, etc. */;
const int kMaxMotorRPM = /* Enter motor cartridge RPM at full power */;

// ----- PID PARAMETERS --------------- //
// Tune these PID constants for both distance and heading control.
const double kP_distance = /* Enter kP value for distance PID */;
const double kI_distance = /* Enter kI value for distance PID */;
const double kD_distance = /* Enter kD value for distance PID */;

const double kP_heading  = /* Enter kP value for heading PID */;
const double kI_heading  = /* Enter kI value for heading PID */;
const double kD_heading  = /* Enter kD value for heading PID */;

// ----- PID Controller Class ------- //
class PIDController {
public:
  PIDController(double kP, double kI, double kD) :
      kP_(kP), kI_(kI), kD_(kD), lastError_(0.0), integral_(0.0) {}


  double calculate(double error, double dt) {
    integral_ += error * dt;
    double derivative = (error - lastError_) / dt;
    lastError_ = error;
    return kP_ * error + kI_ * integral_ + kD_ * derivative;
  }

  void reset() {
    lastError_ = 0.0;
    integral_ = 0.0;
  }

private:
  double kP_, kI_, kD_;
  double lastError_;
  double integral_;
};

// ----- VEX COMPONENTS ------------- //
brain Brain;

motor leftMotor (1, ratio18_1, false);   // Port 1, adjust the gear ratio and reversed status if needed.
motor rightMotor(2, ratio18_1, true);     // Port 2, reversed flag true if required.


// ----- ODOMETRY VARIABLES --------- //  
double currentX = 0.0;
double currentY = 0.0;
double currentAngle = 0.0; // in degrees


void updateOdometry() {
  // Example:
  // currentX = ...;  // calculate using encoder data
  // currentY = ...;
  // currentAngle = InertialSensor.heading();
}

// ----- MOVEMENT FUNCTIONS --------- //
void driveToCoordinate(double targetX, double targetY, double targetAngle) {
  PIDController distancePID(kP_distance, kI_distance, kD_distance);
  PIDController headingPID (kP_heading,  kI_heading,  kD_heading);
  
  // Tolerances can be adjusted (e.g., distance in inches or cm, heading in degrees)
  const double distanceTolerance = 1.0;  // acceptable distance error
  const double angleTolerance    = 2.0;   // acceptable heading error (in degrees)
  
  const double dt = 0.02;  // loop time (20ms typical for control loops)
  
  // Continue until both the distance and heading errors are within tolerance.
  while (true) {
    // Update current robot position and heading
    updateOdometry();
    
    // Compute the error in position
    double deltaX = targetX - currentX;
    double deltaY = targetY - currentY;
    double distanceError = sqrt(deltaX * deltaX + deltaY * deltaY);
    
    // Calculate desired heading from current coordinates (in degrees)
    double desiredHeading = atan2(deltaY, deltaX) * (180.0 / M_PI);
    double headingError = desiredHeading - currentAngle;
    
    // Normalize heading error to range [-180, 180]
    if (headingError > 180) {
      headingError -= 360;
    } else if (headingError < -180) {
      headingError += 360;
    }

    // Use the PID controllers to compute power outputs.
    double drivePower = distancePID.calculate(distanceError, dt);
    double turnPower  = headingPID.calculate(headingError, dt);
    
    // Debug/print statements (optional)
    Brain.Screen.printAt(10, 40, "DistErr: %f", distanceError);
    Brain.Screen.printAt(10, 60, "HeadErr: %f", headingError);
    
    // Set motor speeds:
    // For a simple tank drive, add turn power to one side and subtract it on the other.
    // Adjust the sign and scale if necessary to match your robot's configuration.
    leftMotor.spin(forward, drivePower + turnPower, voltageUnits::volt);
    rightMotor.spin(forward, drivePower - turnPower, voltageUnits::volt);
    
    // Check if within tolerance on both linear and angular error.
    if (distanceError < distanceTolerance && fabs(headingError) < angleTolerance) {
       break; // Target reached
    }
    
    // Wait to maintain dt cycle.
    task::sleep(dt * 1000);
  }
  
  // Stop motors once done.
  leftMotor.stop();
  rightMotor.stop();
}

// ----- TURN FUNCTION (OPTIONAL) ----- //
// If you want a separate routine for turning to a target angle.
void turnToAngle(double targetAngle) {
  PIDController turnPID(kP_heading, kI_heading, kD_heading);
  const double angleTolerance = 2.0; // degrees tolerance
  const double dt = 0.02;
  
  while (true) {
    // It might be useful to update your odometry here if your heading sensor is fused.
    updateOdometry();
    
    double headingError = targetAngle - currentAngle;
    if (headingError > 180) {
      headingError -= 360;
    } else if (headingError < -180) {
      headingError += 360;
    }
    
    double turnPower = turnPID.calculate(headingError, dt);
    
    // For turning, drive motors in opposite directions.
    leftMotor.spin(forward, turnPower, voltageUnits::volt);
    rightMotor.spin(forward, -turnPower, voltageUnits::volt);
    
    if (fabs(headingError) < angleTolerance) {
      break;
    }
    
    task::sleep(dt * 1000);
  }
  
  leftMotor.stop();
  rightMotor.stop();
}

// ----- MAIN PROGRAM -------------- //
int main() {
  
  vexcodeInit();
  
  // Example of commanding the robot to move to coordinates (x, y) and then face a specific angle.
  double targetX = 48.0;      // Replace with your target X coordinate
  double targetY = 0.0;       // Replace with your target Y coordinate
  double targetAngle = 90.0;  // Replace with desired final heading (degrees)
  
  driveToCoordinate(targetX, targetY, targetAngle);
 
  // turnToAngle(targetAngle);
  
  return 0;
}
