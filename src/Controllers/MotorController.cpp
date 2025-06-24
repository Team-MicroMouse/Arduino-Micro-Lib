#include "../algorithms/Algorithms.h" 

void MotorController::Setup() {
    // Initialization code for the robot controller
}

void MotorController::UpdateMoveState(float dt, RobotPosition position) {
    gridPos = position.gridPos;
    gyroAngle = position.angle;
    fhs = fTof->ReadValue();
    fhs = fhs < 255 ? fhs : 0; 

    switch (moveState) {
        case Idle: {
            if (isPaused) {
                lMotor->setThrottle(0);
                rMotor->setThrottle(0);
                return;
            }

            v2f targetGridPosF = (targetGridPos - gridPos).toV2f();
            v2f up = {0, 1};
            float angle = -v2f::signedAngleUp(up, targetGridPosF);
            if (abs(deltaAngleF(targetAngle, angle)) > turnTollerance) { 
                targetAngle = constrain(fmod(angle, 360.0f), 0.0f, 359.0f); 
                moveState = Turning;
                break;
            }

            float distance = (position.position - position.gridPos).length() * 180.0f + 2.0f; // 2cm tolerance
            if (distance > 0.1f) {
                EnterMoving(distance);
            }
            break;
        }
        case Moving: {
            distanceCovered += (position.position.toV2f() - lastPos).length();
            lastPos = position.position.toV2f();

            if (distanceCovered >= targetDistance) {
                EnterIdle();
                break;
            }

            if (fhs < 35 && fhs > 10) {
                EnterIdle();
                break;
            }

            float sideCorrection = 0.0f; //placeholder.

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

            rMotor->setDirection(true); // forward
            lMotor->setDirection(false); // backward

            lMotor->setThrottle(round(outputL));
            rMotor->setThrottle(round(outputR));
            break;
        }
        case Turning: {
            int robotAngle = gyroAngle;
            robotAngle >= 360 ? 0 : robotAngle;
            if (robotAngle > targetAngle - turnTollerance && robotAngle < targetAngle + turnTollerance) {
                EnterIdle();
                break;
            } else {
                float angleError = AngleDifference(robotAngle, targetAngle);

                int throttle = round(abs(angleError));
                throttle = constrain(throttle, 0, lMotor->maxThrotthle());

                int dir = angleError >= 0 ? 1 : 0;
                lMotor->setDirection(dir < 1);
                lMotor->setThrottle(throttle);

                rMotor->setDirection(dir > 0);
                rMotor->setThrottle(throttle);
            }
            break;
        }
    }
}

void MotorController::EnterIdle() {
    moveState = Idle;
    lMotor->setThrottle(0);
    rMotor->setThrottle(0);
}

void MotorController::EnterMoving(float distance) {
    targetDistance = distance;
    distanceCovered = 0.0f;
    moveState = Moving;
}

MotorController::MoveState MotorController::GetMoveState() {
    return moveState;
}

void MotorController::Stop() {
    moveState = Idle;
    lMotor->setThrottle(0);
    rMotor->setThrottle(0);

    isPaused = true;
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
    isPaused = false; 
    targetGridPos = target;
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

float MotorController::deltaAngleF(float a, float b) {
    return fmod((b - a + 540.0f), 360.0f) - 180.0f;
}
