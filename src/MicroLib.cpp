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

Encoder MicroLib::CreateEncoder(uint8_t channelA, uint8_t channelB, uint8_t ppr, float circumference, float interval) {
    Encoder encoder(channelA, channelB, ppr, circumference, interval);
    encoder.Setup();
    return encoder;
}

MotorController MicroLib::motorController(Motor* leftMotor, Motor* rightMotor, TofSensor* leftSensor, TofSensor* rightSensor) {
    MotorController controller;
    controller.SetMotors(leftMotor, rightMotor);
    controller.SetTofSensors(leftSensor, rightSensor);
    return controller;
}

PositionTracker MicroLib::positionTracker(Encoder leftEncoder, Encoder rightEncoder, float cellSize) {
    return PositionTracker(leftEncoder, rightEncoder, cellSize);
}

