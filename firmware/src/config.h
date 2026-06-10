#pragma once

// WiFi
#define WIFI_SSID "KEI-Net"
#define WIFI_PASS "kei-robot-2024"

// Static IP (optional) — comment out to use DHCP
// #define USE_STATIC_IP
#define STATIC_IP IPAddress(192, 168, 4, 1)
#define GATEWAY   IPAddress(192, 168, 4, 1)
#define SUBNET    IPAddress(255, 255, 255, 0)

// Motor pins (L298N via TB6612 or similar H-bridge)
#define MOTOR_L_PWM  32
#define MOTOR_L_IN1  33
#define MOTOR_L_IN2  25
#define MOTOR_R_PWM  26
#define MOTOR_R_IN1  27
#define MOTOR_R_IN2  14

// Motor PWM
#define PWM_FREQ      5000
#define PWM_RES       8
#define PWM_CH_L      0
#define PWM_CH_R      1

// Servo
#define SERVO_PIN     13

// Buzzer (passive)
#define BUZZER_PIN    4
#define BUZZER_PWM_CH  2
#define BUZZER_PWM_FREQ 2000

// I2C
#define I2C_SDA       21
#define I2C_SCL       22

// VL53L0X XSHUT (optional, for multiple sensors)
#define VL53L0X_XSHUT -1   // -1 = not used

// MPU6050 interrupt (optional)
#define MPU_INT_PIN   -1

// Defaults
#define DEFAULT_SAFE_DIST_MM  200
#define DEFAULT_SPEED_LIMIT   255
#define DEFAULT_EXPLORE_SPEED 140
#define TELEMETRY_INTERVAL_MS 500
#define LOG_POLL_MAX          50
