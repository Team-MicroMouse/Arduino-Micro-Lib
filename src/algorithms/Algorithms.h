#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "../util/types/Types.h"

class IRobotController { 
    public:
        virtual ~IRobotController() = default;
        virtual void Setup() = 0;
        virtual void Loop() = 0;
};



class IMotorController { 
    public:
        enum MoveState: int {
            Idle,
            Turning, 
            Moving
        };
    
        virtual ~IMotorController() = default;
        virtual void Setup() = 0;
        virtual void UpdateMoveState(float dt, RobotPosition position) = 0;
        virtual MoveState GetMoveState() = 0;
        virtual float GetDistanceCovered() = 0;
        virtual float GetTargetDistance() = 0;
        virtual void SetGyroNull() = 0;
        virtual void SetRpm(int rpm) = 0;
        virtual void MoveDistance(float distance) = 0;
        virtual void RotateToAngle(int wantedAngle) = 0;
        virtual void RotateDegrees(int degrees) = 0;
};

class IPositionTracker {
    public:
        virtual ~IPositionTracker() = default;
        virtual void Setup() = 0;
        virtual void Process(RobotPosition* RobotPosition) = 0; 
};

class IObjectDetectorAlgorithm {
    public:
        virtual ~IObjectDetectorAlgorithm() = default;
        virtual void Setup() = 0;
        virtual void Process(int* map, v2i mapSize) = 0; 
};

class MotorController : IMotorController {
    public:
        void Setup() override;
        void UpdateMoveState(float dt, RobotPosition position) override;
        MoveState GetMoveState() override;
        float GetDistanceCovered() override;
        float GetTargetDistance() override;
        void SetGyroNull() override;
        void SetRpm(int rpm) override;
        void MoveDistance(float distance) override;
        void RotateToAngle(int wantedAngle) override;
        void RotateDegrees(int degrees) override;

    private:
        MoveState moveState = Idle;
        float distanceCovered = 0.0f;
        float targetDistance = 0.0f;
};

class PositionTracker : IPositionTracker {
    public:
        void Setup() override;
        void Process(RobotPosition* RobotPosition) override;

    private:
        RobotPosition lastPosition;
        float lastDistanceCovered = 0.0f;
};


#endif