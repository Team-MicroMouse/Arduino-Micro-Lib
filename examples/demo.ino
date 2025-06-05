#include "../Micro_Lib.h"

MicroLib robot;

TofSensor lhs;
TofSensor rhs; 

MotorController motorController;
PositionTracker positionTracker;

void setup() {
    wire.begin();
    Serial.begin(115200);


    lhs = robot.CreateTof(0);
    rhs = robot.CreateTof(1);
    Gyro gyro = robot.CreateGyro(0);
    Motor leftMotor = robot.CreateMotor(0, 1);
    Motor rightMotor = robot.CreateMotor(2, 3);

    motorController = robot.motorController(&leftMotor, &rightMotor, &lhs, &rhs);
    positionTracker(0, 1, 2, 3, CELL_SIZE_F);
}

void loop() {
    int lhs = lhs.ReadValue(), rhs = rhs.ReadValue();

    v2f pos = positionTracker.GetGridPos();

    motorController->GetMoveState() == MotorController::Idle ? 
        motorController.MoveDistance(100.0f) : 
        motorController.RotateToAngle(90);

    v2i targetPos = (gridPos + v2f::fromAngle(normalizedAngle(robot.GetGyroAngle()) % 360).normalize()).roundToV2i();
}

auto normalizedAngle = [](int angle) {
        return (angle % 360 + 360) % 360; // always between 0–359
    };

