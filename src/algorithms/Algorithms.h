#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "../util/types/Types.h"
#include "Components.h"

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
            Moving,
            MovingToGridPos
        };
    
        virtual ~IMotorController() = default;
        virtual void Setup() = 0;
        virtual void UpdateMoveState(float dt, RobotPosition position) = 0;
        virtual MoveState GetMoveState() = 0;
        virtual void Stop() = 0;
        virtual float GetDistanceCovered() = 0;
        virtual float GetTargetDistance() = 0;
        virtual void SetGyroNull() = 0;
        virtual void SetRpm(int rpm) = 0;
        virtual void MoveDistance(float distance) = 0;
        virtual void MoveToGridPos(v2i target, float cellSize) = 0;
        virtual void RotateToAngle(int wantedAngle) = 0;
        virtual void RotateDegrees(int degrees) = 0;
};

class IPositionTracker {
    public:
        virtual ~IPositionTracker() = default;
        virtual void Setup() = 0;
        virtual void Process() = 0;
        virtual void SetGyro(Gyro gyro1) = 0;
        virtual void ResetDistance() = 0;
        virtual void ResetPosition() = 0;
        virtual void ResetGyro() = 0; 
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
        void Stop() override;
        float GetDistanceCovered() override;
        float GetTargetDistance() override;
        void SetGyroNull() override;
        void SetRpm(int rpm) override;
        void MoveDistance(float distance) override;
        void MoveToGridPos(v2i target, float cellSize) override;
        void RotateToAngle(int wantedAngle) override;
        void RotateDegrees(int degrees) override;

        void SetMotors(Motor* leftMotor, Motor* rightMotor) {
            lMotor = leftMotor;
            rMotor = rightMotor;
        }

        void SetTofSensors(TofSensor* leftSensor, TofSensor* rightSensor, TofSensor* frontSensor) {
            lTof = leftSensor;
            rTof = rightSensor;
            fTof = frontSensor;
        }

        v3f pid {0.0f, 0.0f, 0.0f};
        v3f pidSide {0.0f, 0.0f, 0.0f};

    private:
        float SideCorrection(float dt, int lSensor, int rSensor);
        void EnterIdle();
        void EnterMoving(float distance);
        float AngleDifference(int fa, int b);
        float deltaAngleF(float a, float b);

        MoveState moveState = Idle;
        Motor* lMotor = nullptr;
        Motor* rMotor = nullptr;
        TofSensor* lTof = nullptr;
        TofSensor* rTof = nullptr;
        TofSensor* fTof = nullptr;
        uint8_t fhs = 0, lhs = 0, rhs = 0;
        bool isPaused = false;
        int gyroAngle = 0; // degrees
        float turnTollerance = 0.5f; // degrees
        int targetAngle = 0;
        v2f lastPos = {0.0f, 0.0f};
        v2i gridPos = {0, 0};
        v2i targetGridPos = {0, 0};
        float distanceCovered = 0.0f;
        float targetDistance = 0.0f;
        int targetRpm = 0, wantedRpm = 0;
        float prevErrorR, prevErrorL;
        float integralR, integralL;
        float integralSat = 1.0f;
        float prevSideError, integralSide;
        
};

class PositionTracker : IPositionTracker {
    public:
        void Setup() override;
        void SetEncoders(Encoder left, Encoder right, float cellSize) {
            leftEncoder = left;
            rightEncoder = right;
            this->cellSize = cellSize;
        }
        void Process() override;
        void SetGyro(Gyro gyro1) override;
        void ResetDistance() override;
        void ResetPosition() override;
        void ResetGyro() override;


        RobotPosition GetPosition() const;
        float GetDistanceCovered() const;
        v2f GetGridPos() const;
    private:
        RobotPosition robot;
        v2i lastPosition;
        float cellSize;
        float distance = 0.0f;

        Gyro* gyro = nullptr;
        Encoder leftEncoder, rightEncoder;
        uint8_t LA, LB, RA, RB;
};


#endif