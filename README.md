## Arduino Micro Lib

This library has been designed for use within the Arduino IDE but will most likely also work for Platform IO. This library utalizes the same logic and functions as the Unity Micro Lib which allows for easy copying of algoritms designed in the simulation. 

## Technical stuff

### Requirments
- A variant of an ESP32 (most Arduino boards won't work due to hardware limitation). For the best results use either an ESP32-S3 or ESP32C6 as this libary has been developed on these boards.
- Support for I2C. 
- Access to IRAM_ATTR or internal memory of the chip. 

### Supported parts
Since this library is case made for our robot we only support a limited amount sensors. Currently we're not planning on adding support for more, but feel free to make a pr or fork to add your own. 

Supported sensors/chips: 
- VL6180(x) (ToF)
- MPU5060 (accelerometer/gyro)
- SSD1306 (oled screen)
- TCA9548A (multiplexer)
- Any quadrature (magnetic) encoder

### Setting up the library

After you have [installed](https://support.arduino.cc/hc/en-us/articles/5145457742236-Add-libraries-to-Arduino-IDE) the library make use of the following example to get started. 
Here's a simple example on a setup of a simple robot. 
```cpp
#include <MicroLib.h>
#include <Wire.h>
#include <Arduino.h>


#define INTERVAL 40
#define CELLSIZE 180.0f

// defines robot controllers 
MicroLib robot;

MotorController motorController;
PositionTracker tracker;

// defines sensors
TofSensor lhs;
TofSensor rhs;
TofSensor fhs;
Gyro gyro;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // assigns pins to motors
  Motor leftMotor = robot.CreateMotor(0, 1);
  Motor rightMotor = robot.CreateMotor(2, 3);

  // assigns encoders and intervals
  Encoder leftEncoder = robot.CreateEncoder(10, 11, 7, 180.0f, INTERVAL);
  Encoder rightEncoder = robot.CreateEncoder(12, 13, 7, 180.0f, INTERVAL);
  
  // assigns i2c multiplexer channels to i2c based sensors
  lhs = robot.CreateTof(0);
  rhs = robot.CreateTof(4);
  fhs = robot.CreateTof(1);
  gyro = robot.CreateGyro(6);

  // initializes controllers
  motorController = robot.motorController(&leftMotor, &rightMotor, &lhs, &lhs, &fhs);
  motorController.pid = {0.7f, 0.54f, 0.05f};
  tracker = robot.positionTracker(leftEncoder, rightEncoder, gyro, CELLSIZE);
}

void loop() {
  // basic update loop, put your algoritm under this section
  tracker.Process();
  motorController.UpdateMoveState(INTERVAL, tracker.GetPosition())
}
```

### Additional types 

This library also comes with some extra types to smoothen the creation of MicroMouse algorithms. 

- V2I: Vector2Int structure with basic operators and a few vector methods
- V2F: Vector2Float structure with a few conversion types, operators and vector transformations
- V3I: Vector3Int similar to V2I
- V3F: Vector3Float similar to V2F
- RobotPosition: stores a v2i for the raw position and gridposition and current angle of the robot.
- MapCell: includes functions to set, discover and highlight walls within a cell of the maze. 
- Map: contains a list of all the MapCell's within a maze. 
- Some trigonometry function such as RAD2DEG, DEG2RAD, PI

