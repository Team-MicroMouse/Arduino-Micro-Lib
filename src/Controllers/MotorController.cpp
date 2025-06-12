#include "../algorithms/Algorithms.h" 

void MotorController::Setup() {
    // Initialization code for the robot controller
}

void MotorController::UpdateMoveState(float dt, RobotPosition position) {
    gyroAngle = position.angle; 

    switch(moveState) {
        case Moving:
            if (distanceCovered >= targetDistance) {
                moveState = Idle;
            } 
            break;

        case Turning:
            if (abs(prevErrorT) < 0.2f) {
                moveState = Idle;
            }
            break;
        case MovingToGridPos: 
            if (abs(prevErrorT) < 0.2f) {
                if (distanceToGrid < 0.1f) moveState = Idle;

                MoveDistance(distanceToGrid);
            }
            break;
        case Idle:
            wantedRpm = 0; 
            break;
    }
}

MotorController::MoveState MotorController::GetMoveState() {
    return moveState;
}

void MotorController::Stop() {
    moveState = Idle;
    wantedRpm = 0;
    
    lMotor->SetThrottle(0);
    rMotor->SetThrottle(0);
}

float MotorController::GetDistanceCovered() {
    return distanceCovered;
}

float MotorController::GetTargetDistance() {
    return targetDistance;
}

void MotorController::SetGyroNull() {
    gyroAngle = 0;
}

void MotorController::SetRpm(int rpm) {
    wantedRpm = constrain(rpm, 0, 255);
}

void MotorController::MoveDistance(float distance) {
    targetDistance = distance;
    moveState = Moving;
}

void MotorController::MoveToGridPos(v2i target, float cellSize) {
    v2f targetPos = v2f(target.x * cellSize, target.y * cellSize);
    v2f direction (targetPos - position.gridPos).normalize();

    float angle = signedAngle(direction) * RAD2DEG;
    RotateToAngle((int)angle);

    distanceToGrid = (targetPos - position.position).length();
    MoveState = MovingToGridPos;
}

void MotorController::RotateToAngle(int wantedAngle) {
    targetAngle = wantedAngle % 360;
    SetRpm(40); 
    moveState = Turning;
}

void MotorController::RotateDegrees(int degrees) {
    int new_angle = targetAngle + degrees;
    if (new_angle >= 360) {
        new_angle -= 360;
    } else if (new_angle < 0) {
        new_angle = 360 - new_angle;
    }    

    targetAngle = new_angle;
    SetRpm(40);
    moveState = Turning;
}

void MotorController::UpdateMotor(float dt, RobotPosition position) {
    this->position = position;
    if (targetRpm <= 0) return;

    switch(moveState) {
        case Turning: {
            float errorT = AngleDifference(gyroAngle, targetAngle);
            integralT = constrain(integralT + errorT * dt, -integralTSat, integralTSat);
            float derivativeT = (errorT - prevErrorT) / dt;
            float outputT = pid.z * errorT + pid.y * integralT + pid.x * derivativeT; 
            prevErrorT = errorT;

            int throttle = round(abs(outputT));
            int dir = outputT >= 0 ? 1: -1;

            lMotor->SetThrottle(-dir * throttle);
            rMotor->SetThrottle(dir * throttle);
            }
            break;
        case Moving: {
            float sideCorrection = SideCorrection(dt, position.position.x, position.position.y);


            float errorL = wantedRpm - lMotor->currentThrottle();
            integralL = constrain(integralL + errorL * dt, -integralSat, integralSat);
            float derivativeL = (errorL - prevErrorL) / dt;
            float outputL = pid.x * errorL + pid.y * integralL + pid.x * derivativeL; 
            outputL = constrain(outputL - sideCorrection, -lMotor->maxThrotthle(), lMotor->maxThrotthle());
            prevErrorL = errorL;

            float errorR = wantedRpm - rMotor->currentThrottle();
            integralR = constrain(integralR + errorR * dt, -integralSat, integralSat);
            float derivativeR = (errorR - prevErrorR) / dt;
            float outputR = pid.x * errorR + pid.y * integralR + pid.z * derivativeR;
            outputR = constrain(outputR + sideCorrection, -rMotor->maxThrotthle(), rMotor->maxThrotthle());
            prevErrorR = errorR;

            lMotor->SetThrottle(round(outputL));
            rMotor->SetThrottle(round(outputR));
            }   
            break;
        case Idle:{
            prevErrorL = 0.0f;
            prevErrorR = 0.0f;
            prevErrorT = 0.0f;
            }
            break;
    }
}

float MotorController::SideCorrection(float dt, int lSensor, int rSensor) {
    float error = lSensor - rSensor;
    if (abs(error) < 5 || abs(error) > 35) {
        return 0.0f; 
    }
    error = constrain(error, -22.0f, 22.0f);
    
    integralSide = constrain(integralSide + error * dt, -integralSat, integralSat);
    float derivative = (error - prevSideError) / dt;
    float output = pidSide.x * error + pidSide.y * integralSide + pidSide.z * derivative;
    prevSideError = error;

    return output;
}

float MotorController::AngleDifference(int a, int b) {
    return (a- b + 540) % 360 - 180;
}
