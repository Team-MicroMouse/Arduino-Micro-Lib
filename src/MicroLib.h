#ifndef MICROLIB_H
#define MICROLIB_H

#include "algorithms/Components.h"
#include "algorithms/Algorithms.h"
#include "util/types/Types.h"

class MicroLib {
public:
    ~MicroLib() = default;
    void update();

    TofSensor& CreateTof(uint8_t channel, const char* which);
    Gyro& CreateGyro(uint8_t channel);
    Motor& CreateMotor(uint8_t pwmChannel, uint8_t dirChannel, const char* which);
    Encoder& CreateEncoder(uint8_t channelA, uint8_t channelB, float ppr, float circumference, float interval, const char* which);

    MotorController& motorController(Motor* leftMotor, Motor* rightMotor, TofSensor* leftSensor, TofSensor* rightSensor, TofSensor* frontSensor);
    PositionTracker& positionTracker(Encoder& leftEncoder, Encoder& rightEncoder, Gyro& gyro, float cellSize);

private:
    MotorController motorCtrl;
    PositionTracker posTracker;
    
    Motor leftMotor;
    Motor rightMotor;

    Encoder leftEncoder;
    Encoder rightEncoder;

    TofSensor lhs;
    TofSensor rhs;
    TofSensor fhs;
    Gyro gyro;
};

#endif