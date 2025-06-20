#ifndef MICROLIB_H
#define MICROLIB_H

#include "algorithms/Components.h"
#include "algorithms/Algorithms.h"
#include "util/types/Types.h"

class MicroLib {
public:
    ~MicroLib() = default;
    void update();

    TofSensor CreateTof(uint8_t channel);
    Gyro CreateGyro(uint8_t channel);
    Motor CreateMotor(uint8_t pwmChannel, uint8_t dirChannel);
    Encoder CreateEncoder(uint8_t channelA, uint8_t channelB, uint8_t ppr, float circumference, float interval);
    
    MotorController motorController(Motor* leftMotor, Motor* rightMotor, TofSensor* leftSensor, TofSensor* rightSensor);
    PositionTracker positionTracker(Encoder leftEncoder, Encoder rightEncodeer, float cellSize);
};

#endif
