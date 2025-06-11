#include "../algorithms/Algorithms.h" 

void PositionTracker::Setup() {
    leftEncoder.Setup();
    rightEncoder.Setup();
    robot.position = v2i(0, 0);
    robot.angle = 0;
    distance = 0.0f;
}

void PositionTracker::SetGyro(Gyro* gyro) {
    this->gyro = gyro;
}

void PositionTracker::Process() {
    
    float angle = gyro ? gyro->ReadValue() : 0.0f;
    float movement = (leftEncoder.distance + rightEncoder.distance) / 2.0f;

    float dx = movement * cos(angle * DEG2RAD);
    float dy = movement * sin(angle * DEG2RAD);

    robot.position = robot.position + v2i(dx, dy);
    robot.angle = static_cast<int>(angle) % 360;
    distance += movement;
}

float PositionTracker::GetDistanceCovered() const {
    return distance;
}

v2f PositionTracker::GetGridPos() const {
    return (robot.position / cellSize).round();
}

void PositionTracker::ResetPosition() {
    robot.position = v2i(0, 0);
    distance = 0.0f;
}

void PositionTracker::ResetGyro() {
    if (gyro) {
        gyro->Reset();
    }
}