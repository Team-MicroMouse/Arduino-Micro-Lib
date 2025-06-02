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
                wantedRpm = 0;
            } 
            break;

        case Turning:
            if (abs(prevErrorT) < 0.2f) {
                moveState = Idle;
                wantedRpm = 0;
            }
            break;
    }
}

MotorController::MoveState MotorController::GetMoveState() {
    return moveState;
}

void MotorController::Stop() {
    moveState = Idle;
    wantedRpm = 0;
    
    // Set throttle to zero for both motors
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
    // Command the robot to move a specific distance
    targetDistance = distance;
    moveState = Moving;
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
    if (targetRpm <= 0) return;

    switch(moveState) {
        case Turning:
            float errorT = AngleDifference(gyroAngle, targetAngle);
            integralT = constrain(integralT + errorT * dt, -integralTSat, integralTSat);
            float derivativeT = (errorT - prevErrorT) / dt;
            float outputT = pid.z * errorT + pid.y * integralT + pid.x * derivativeT; // add pid variables here
            prevErrorT = errorT;

            int throttle = round(abs(outputT));
            int dir = outputT >= 0 ? 1: -1;

            //set motor throttle.
            break;
        case Moving: 
            float sideCorrection = SideCorrection(dt, position.position.x, position.position.y);


            float errorL = wantedRpm - lMotor.CurrentRPM();
            integralL = constrain(integralL + errorL * dt, -integralSat, integralSat);
            float derivativeL = (errorL - prevErrorL) / dt;
            float outputL = pid.x * errorL + pid.y * integralL + pid.x * derivativeL; 
            outputL = constrain(outputL - sideCorrection, -maxRpm, maxRpm);
            prevErrorL = errorL;

            float errorR = wantedRpm - rMotor.CurrentRPM();
            integralR = constrain(integralR + errorR * dt, -integralSat, integralSat);
            float derivativeR = (errorR - prevErrorR) / dt;
            float outputR = pid.x * errorR + pid.y * integralR + pid.z * derivativeR;
            outputR = constrain(outputR + sideCorrection, -maxRpm, maxRpm);
            prevErrorR = errorR;

            //set motor throttle.
            break;
        case Idle:
            prevErrorL = 0.0f;
            prevErrorR = 0.0f;
            prevErrorT = 0.0f;
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
    float output = pidSide.x * error + pidSide.y * integralSide + pidSide.z * derivative; // add pid variables here
    prevSideError = error;

    return output;
}

float AngleDifference(int a, int b) {
    return (a- b + 540) % 360 - 180;
}
