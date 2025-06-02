#include "Components.h"

void MPSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(0x70); // Multiplexer I2C address
    Wire.write(1 << channel); 
    Wire.endTransmission();
}

bool Gyro::Setup(uint8_t channel) {
    this->channel = channel;
    delay(5);
    MPSelect(channel);
    bool success = sensor.begin();
    bias = calibrateGyro(250);
    return success;
}

float CalibrateGyro(int samples) {
    long sum = 0; 
    for (int i = 0; i < samples; i++) {
      sensors_event_t a, g, temp;
      sensor.getEvent(&a, &g, &temp);  
      sum += g.gyro.z;
      delay(1);
    }
    float res = (float)sum / samples / 131.0f;
    return res;
}

uint8_t Gyro::ReadValue() {
    MPSelect(channel);
    delay(1);
    sensors_event_t a, g, temp;
    sensor.getEvent(&a, &g, &temp);

    float gyroZ = g.gyro.z * 180.0f / PI;
    gyroZ -= bias;
    float dz = deadzone * 180.0f / PI;
    if (abs(gyroZ) < dz) gyroZ = 0.0f;

    unsigned long now = micros();
    float dt = (now - lastTime) / 1e6f;
    lastTime = now;

    totalAngle += gyroZ * dt;
    totalAngle = wrap360(totalAngle);

    return (int)totalAngle;
}

uint8_t Gyro::ReadStatus() {
    return 0; // Placeholder for actual status reading
}

bool TofSensor::Setup(uint8_t channel) {
    this->channel = channel;
    delay(5);
    MPSelect(channel);
    return sensor.begin();
}

uint8_t TofSensor::ReadValue() {
    MPSelect(channel);
    delay(1);
    return sensor.readRange();
}

uint8_t TofSensor::ReadStatus() {
    MPSelect(channel);
    delay(1);
    return sensor.readRangeStatus();
}

float wrap360(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

Encoder::Encoder(uint8_t channelA, uint8_t channelB, float ppr, float circumference, float interval)
    : channelA(channelA), channelB(channelB), ppr(ppr), circumference(circumference), interval(interval){}

void Encoder:: PulseCounter() {
    pulses += (digitalRead(channelA)) ? 1 : -1;
}

void Encoder::Setup() {
    pinMode(channelA, INPUT);
    pinMode(channelB, INPUT);

    attachInterrupt(digitalPinToInterrupt(channelB), PulseCounter, RISING);

    speed = 0.0f;
    distance = 0.0f;
}

void Encoder::Process() {
    distance = (pulses / ppr) * circumference; // in cm
    speed = (pulses / ppr) * (1000.0f / interval) * 60; 

    pulses = 0; // Reset pulses after processing
}

void Motor::Setup(uint8_t pwmChannel, uint8_t dirChannel) {
    this->pwmChannel = pwmChannel;
    this->dirChannel = dirChannel;
    pinMode(pwmChannel, OUTPUT);
    pinMode(dirChannel, OUTPUT);
}

int Motor::currentThrottle() {
    return throttle;
}

void Motor::SetThrottle(int throttle) {
    this->throttle = constrain(throttle, 0, maxThrottle);
    digitalWrite(dirChannel, (throttle >= 0) ? HIGH : LOW);
    analogWrite(pwmChannel, abs(throttle));
}

int Motor::maxThrotthle() {
    return maxThrottle;
}


