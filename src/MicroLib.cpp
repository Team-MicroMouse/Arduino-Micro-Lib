#include "MicroLib.h"

TofSensor MicroLib::CreateTof(uint8_t channel) {
    TofSensor sensor;
    if (sensor.Setup(channel)) {
        return sensor;
    }
    return TofSensor(); 
}

Gyro MicroLib::CreateGyro(uint8_t channel) {
    Gyro sensor;
    if (sensor.Setup(channel)) {
        return sensor;
    }
    return Gyro(); 
}

Motor MicroLib::CreateMotor(uint8_t pwmChannel, uint8_t dirChannel) {
    Motor motor;
    motor.Setup(pwmChannel, dirChannel);
    return motor;
}

MotorController MicroLib::motorController(Motor* leftMotor, Motor* rightMotor, TofSensor* leftSensor, TofSensor* rightSensor) {
    MotorController controller;
    controller.SetMotors(leftMotor, rightMotor);
    controller.SetTofSensors(leftSensor, rightSensor);
    return controller;
}

PositionTracker MicroLib::positionTracker(uint8_t LA, uint8_t LB, uint8_t RA, uint8_t RB, float cellSize) {
    PositionTracker tracker(cellSize);
    tracker.SetEncoderPins(LA, LB, RA, RB);
    return tracker;
}

