#ifndef MICROLIB_H
#define MICROLIB_H

#include "algorithms/Components.h"
#include "algorithms/Algorithms.h"
#include "util/types/Types.h"

class MicroLib {
public:
    ~MicroLib() = default;

    TofSensor CreateTof(uint8_t channel);
    Gyro CreateGyro(uint8_t channel);
    Motor CreateMotor(uint8_t pwmChannel, uint8_t dirChannel);
    
    MotorController motorController(Motor* leftMotor, Motor* rightMotor, TofSensor* leftSensor, TofSensor* rightSensor);
    PositionTracker positionTracker(uint8_t LA, uint8_t LB, uint8_t RA, uint8_t RB, float cellSize);
};

#endif
