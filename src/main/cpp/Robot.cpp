/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <frc/ADXRS450_Gyro.h>

#include <ctre/Phoenix.h>

// NOTE(Tyler): Despite programming in C++ the style of programming is more closely related to plain C.
// This means that templates are to be used in almost NO circumstances, as well as no smart pointers. This
// is similar to the style used by Casey Muratori in Handmade Hero. C++ features are only used when 
// absolutely necessary such as using WPILib or the Phoenix framework, and even then C++ features
// aren't fully used. Programming in C style C++ allows for a simpler program which means greater 
// maintainability and greater debuggability.

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

Joystick global_gamepad = Joystick(0);

WPI_TalonSRX global_front_left_drive = WPI_TalonSRX(4); // master
WPI_TalonSRX global_rear_left_drive = WPI_TalonSRX(3);
WPI_TalonSRX global_front_right_drive = WPI_TalonSRX(1);
WPI_TalonSRX global_rear_right_drive = WPI_TalonSRX(2); // master

DriveTrain global_drive_train = {global_front_left_drive, global_rear_right_drive};

VictorSPX global_intake_motor = VictorSPX(3);
DoubleSolenoid global_intake_solenoid = DoubleSolenoid(4, 3);
VictorSPX global_uptake_motor = VictorSPX(1);
b32 global_is_intake_toggled;
b32 global_is_intake_raised = true; // The intake starts out raised

TalonSRX global_left_shooter_motor = TalonSRX(7);
TalonSRX global_right_shooter_motor = TalonSRX(6);

Compressor global_compressor = Compressor();

ADXRS450_Gyro global_gyro = ADXRS450_Gyro();

void Robot::RobotInit() {
    global_front_left_drive.SetNeutralMode(NeutralMode::Brake);
    global_rear_left_drive.SetNeutralMode(NeutralMode::Brake);
    global_front_right_drive.SetNeutralMode(NeutralMode::Brake);
    global_rear_right_drive.SetNeutralMode(NeutralMode::Brake);

    global_compressor.Start();
    global_compressor.SetClosedLoopControl(true);
}

void Robot::TeleopPeriodic() {
    f32 power = global_gamepad.GetRawAxis(RIGHT_Y_AXIS);
    f32 power = global_gamepad.GetRawAxis(RIGHT_X_AXIS)*0.75;
    

    if(global_gamepad.GetRawButtonPressed(START_BUTTON)) {
        global_is_intake_toggled = !global_is_intake_toggled;
    }

    if(global_is_intake_toggled) {
        global_intake_motor.Set(ControlMode::PercentOutput, -0.75);
    } else {
        global_intake_motor.Set(ControlMode::PercentOutput, 0.0);
    }
    
    if(global_gamepad.GetRawButtonPressed(BACK_BUTTON)) {
        if(global_is_intake_raised) {
            global_intake_solenoid.Set(DoubleSolenoid::kForward);
        } else {
            global_intake_solenoid.Set(DoubleSolenoid::kReverse);
        }
        global_is_intake_raised = !global_is_intake_raised;
    }

    if(global_gamepad.GetRawButton(A_BUTTON)) {
        global_uptake_motor.Set(ControlMode::PercentOutput, 1.0);
    } else if(global_gamepad.GetRawButton(B_BUTTON)) {
        global_uptake_motor.Set(ControlMode::PercentOutput, -1.0);
    } else {
        global_uptake_motor.Set(ControlMode::PercentOutput, 0.0);
    }

    f64 shooter_power = global_gamepad.GetRawAxis(RIGHT_TRIGGER);
    global_left_shooter_motor.Set(ControlMode::PercentOutput, -shooter_power);
    global_right_shooter_motor.Set(ControlMode::PercentOutput, shooter_power);
}

void Robot::AutonomousInit() {
    global_gyro.Calibrate();
}

void Robot::AutonomousPeriodic() {
    printf("Angle = %f\n", global_gyro.GetAngle());
}

#ifndef RUNNING_FRC_TESTS
int main() { return(frc::StartRobot<Robot>()); }
#endif
