#include "MicroLib.h"

void MicroLib::update() {
}

TofSensor& MicroLib::CreateTof(uint8_t channel, const char* which) {
    if (strcmp(which, "left") == 0) {
        lhs.Setup(channel);
        return lhs;
    } else if (strcmp(which, "right") == 0) {
        rhs.Setup(channel);
        return rhs;
    } else {
        fhs.Setup(channel);
        return fhs;
    }
}

Gyro& MicroLib::CreateGyro(uint8_t channel) {
    gyro.Setup(channel);
    return gyro;
}

Motor& MicroLib::CreateMotor(uint8_t pwmChannel, uint8_t dirChannel, const char* which) {
    if (strcmp(which, "left") == 0) {
        leftMotor.Setup(pwmChannel, dirChannel);
        return leftMotor;
    } else {
        rightMotor.Setup(pwmChannel, dirChannel);
        return rightMotor;
    }
}

Encoder& MicroLib::CreateEncoder(uint8_t channelA, uint8_t channelB, float ppr, float circumference, float interval, const char* which) {
    if (strcmp(which, "left") == 0) {
        leftEncoder = Encoder(channelA, channelB, ppr, circumference, interval);
        leftEncoder.Setup();
        return leftEncoder;
    } else {
        rightEncoder = Encoder(channelA, channelB, ppr, circumference, interval);
        rightEncoder.Setup();
        return rightEncoder;
    }
}

MotorController& MicroLib::motorController(Motor* leftMotor, Motor* rightMotor, TofSensor* leftSensor, TofSensor* rightSensor, TofSensor* frontSensor) {
    motorCtrl.SetMotors(leftMotor, rightMotor);
    motorCtrl.SetTofSensors(leftSensor, rightSensor, frontSensor);
    motorCtrl.Setup();
    return motorCtrl;
}

PositionTracker& MicroLib::positionTracker(Encoder& leftEncoder, Encoder& rightEncoder, Gyro& gyro, float cellSize) {
    posTracker.SetEncoders(leftEncoder, rightEncoder, cellSize);
    posTracker.SetGyro(gyro);
    posTracker.Setup();
    return posTracker;
}
