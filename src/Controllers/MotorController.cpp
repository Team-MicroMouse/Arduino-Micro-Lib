#include "../algorithms/Algorithms.h"

void MotorController::Setup() {
    // Initialization code for the robot controller
}

void MotorController::UpdateMoveState(float dt, RobotPosition position) {
    // Update the move state based on the elapsed time and current position
}

MotorController::MoveState MotorController::GetMoveState() {
    return moveState;
}

float MotorController::GetDistanceCovered() {
    return distanceCovered;
}

float MotorController::GetTargetDistance() {
    return targetDistance;
}

void MotorController::SetGyroNull() {
    // Reset the gyro sensor to a null state
}

void MotorController::SetRpm(int rpm) {
    // Set the RPM of the motors
}

void MotorController::MoveDistance(float distance) {
    // Command the robot to move a specific distance
    targetDistance = distance;
    moveState = Moving;
}

void MotorController::RotateToAngle(int wantedAngle) {
    // Command the robot to rotate to a specific angle
    moveState = Turning;
}

void MotorController::RotateDegrees(int degrees) {
    // Command the robot to rotate a specific number of degrees
    int wantedAngle = (degrees + 360) % 360; // Normalize angle
    RotateToAngle(wantedAngle);
}

