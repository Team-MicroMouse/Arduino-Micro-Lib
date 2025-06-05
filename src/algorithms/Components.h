#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "../util/types/Types.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_VL6180X.h>
#include <Adafruit_Sensor.h>

class ISensor { 
    public:
        virtual ~ISensor() = default;
        virtual bool Setup(uint8_t channel) = 0;
        virtual uint8_t ReadValue() = 0;
        virtual uint8_t ReadStatus() = 0;
    private:
        uint8_t channel;
};

class IMotor {
    public: 
        virtual ~IMotor() = default;
        virtual void Setup(uint8_t pwmChannel, uint8_t dirChannel) = 0;
        virtual int currentThrottle() = 0;
        virtual void SetThrottle(int throttle) = 0;
        virtual int maxThrotthle() = 0;
    private: 
        uint8_t pwmChannel;
        uint8_t dirChannel;
};

class Gyro : ISensor { 
    public:
        ~Gyro() = default;
        bool Setup(uint8_t channel) override;
        uint8_t ReadValue() override;
        uint8_t ReadStatus() override;
        void Reset();
    private: 
        float calibrateGyro(int samples);

        uint8_t channel;
        Adafruit_MPU6050 sensor;
        float totalAngle = 0.0f;
        unsigned long lastTime = 0;
        float bias = 0.0f;
        float deadzone = 0.1f;
};

class TofSensor : ISensor {
    public:
        virtual ~TofSensor() = default;
        virtual bool Setup(uint8_t channel) override;
        virtual uint8_t ReadValue() override;
        virtual uint8_t ReadStatus() override;
    private:
        uint8_t channel;
        Adafruit_VL6180X sensor;
}; 

class Encoder {
    public:
        Encoder(uint8_t channelA, uint8_t channelB, float ppr, float circumference, float interval);
        void Setup();
        void Process();

        float speed;
        float distance;
    private:
        void PulseCounter();

        int pulses = 0;
        uint8_t channelA;
        uint8_t channelB;
        float ppr; 
        float circumference;
        float interval;
};

class Motor: IMotor {
    public:
        virtual ~Motor() = default;
        virtual void Setup(uint8_t pwmChannel, uint8_t dirChannel) override;
        int currentThrottle() override;
        void SetThrottle(int throttle) override;
        int maxThrotthle() override;

    private:
        uint8_t pwmChannel;
        uint8_t dirChannel;
        int throttle = 0;
        int maxThrottle = 255;
};

void MPSelect(uint8_t channel);

#endif