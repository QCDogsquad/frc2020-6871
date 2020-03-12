/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
// #include <frc/ADXRS450_Gyro.h>

using namespace frc;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef u8 b8;
typedef u16 b16;
typedef u32 b32;
typedef u64 b64;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

typedef float f32;
typedef double f64;

#define LEFT_X_AXIS 0
#define LEFT_Y_AXIS 1
#define RIGHT_X_AXIS 4
#define RIGHT_Y_AXIS 5
#define LEFT_TRIGGER 2
#define RIGHT_TRIGGER 3

#define A_BUTTON 1
#define B_BUTTON 2
#define X_BUTTON 3
#define Y_BUTTON 4

#define LEFT_BUMPER 5
#define RIGHT_BUMPER 6
#define BACK_BUTTON 7
#define START_BUTTON 8
#define LEFT_STICK_BUTTON 9
#define RIGHT_STICK_BUTTON 10

#define POV_UP 0
#define POV_RIGHT 90
#define POV_DOWN 180
#define POV_LEFT 270

struct Robot : public frc::TimedRobot {
    void RobotInit();
    void TeleopPeriodic();
    void AutonomousInit();
    void AutonomousPeriodic();
};
