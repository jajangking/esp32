#pragma once

// WiFi
#define WIFI_SSID "KEI-Net"
#define WIFI_PASS "kei-robot-2024"

// Static IP (optional) — comment out to use DHCP
// #define USE_STATIC_IP
#define STATIC_IP IPAddress(192, 168, 4, 1)
#define GATEWAY   IPAddress(192, 168, 4, 1)
#define SUBNET    IPAddress(255, 255, 255, 0)

// Motor pins (TB6612 H-bridge)
#define MOTOR_L_PWM  25
#define MOTOR_L_IN1  26
#define MOTOR_L_IN2  27
#define MOTOR_R_PWM  13
#define MOTOR_R_IN1  14
#define MOTOR_R_IN2  33
#define MOTOR_STBY   32

// Motor PWM
#define PWM_FREQ      5000
#define PWM_RES       8
#define PWM_CH_L      0
#define PWM_CH_R      1

// Motor ramp (ticks between speed steps)
#define RAMP_STEP_MS  8
#define RAMP_STEP     8

// Motor safety timeout (ms without cmd → stop)
#define MOTOR_TIMEOUT 2000

// Servo
#define SERVO_PIN     5

// Buzzer (passive)
#define BUZZER_PIN    4
#define BUZZER_PWM_CH 2
#define BUZZER_PWM_FREQ 2000

// LED status indicator
#define LED_PIN       2

// I2C
#define I2C_SDA       21
#define I2C_SCL       22

// VL53L0X
#define VL53L0X_XSHUT 15
#define VL_TIMING_BUDGET_DEFAULT 33000
#define VL_SIGNAL_OK_THRESHOLD  0.5f
#define VL_AMBIENT_OK_THRESHOLD 10.0f

// MPU6050 interrupt (optional)
#define MPU_INT_PIN   -1

// Servo sector angles (left, forward, right)
#define SECTOR_L      160
#define SECTOR_F      90
#define SECTOR_R      20

// Defaults
#define DEFAULT_SAFE_DIST_MM  200
#define DEFAULT_SPEED_LIMIT   255
#define DEFAULT_EXPLORE_SPEED 140
#define TELEMETRY_INTERVAL_MS 250
#define LOG_POLL_MAX          50

// IMU thresholds
#define TILT_THRESHOLD_ON    60.0f
#define TILT_THRESHOLD_OFF   45.0f
#define COLLISION_THRESHOLD  14.0f
