/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// NOTE(Tyler): Despite programming in C++ the style of programming is more closely related to plain C.
// This means that templates are to be used in almost NO circumstances, as well as no smart pointers. This
// is similar to the style used by Casey Muratori in Handmade Hero. C++ features are only used when 
// absolutely necessary such as using WPILib or the Phoenix framework, and even then C++ features
// aren't fully used. Programming in C style C++ allows for a simpler program which means greater 
// maintainability and greater debuggability.

#include "Robot.h"

#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <ctre/Phoenix.h>

#define ENABLE_PID_TUNING 0

Joystick global_gamepad = Joystick(0);

VictorSPX global_intake_motor = VictorSPX(3);
DoubleSolenoid global_intake_solenoid = DoubleSolenoid(4, 3);
VictorSPX global_uptake_motor = VictorSPX(1);
b32 global_is_intake_toggled;
b32 global_is_intake_raised = true; // The intake starts out raised

//  front left: 4 master
//  rear  left: 3
// front right: 1
// rear  right: 2 master
WPI_TalonSRX global_front_left_drive = WPI_TalonSRX(4);
WPI_TalonSRX global_rear_left_drive = WPI_TalonSRX(3);
WPI_TalonSRX global_front_right_drive = WPI_TalonSRX(1);
WPI_TalonSRX global_rear_right_drive = WPI_TalonSRX(2);

SpeedControllerGroup global_left_drive = SpeedControllerGroup(global_front_left_drive, global_rear_left_drive);
SpeedControllerGroup global_right_drive = SpeedControllerGroup(global_front_right_drive, global_rear_right_drive);

DifferentialDrive global_robot_drive = DifferentialDrive(global_left_drive, global_right_drive);

TalonSRX global_left_shooter_motor = TalonSRX(7);
TalonSRX global_right_shooter_motor = TalonSRX(6);

Compressor global_compressor = Compressor();

#if ENABLE_PID_TUNING
nt::NetworkTableEntry global_shooter_f_gain;
nt::NetworkTableEntry global_shooter_p_gain;
#endif

void Robot::RobotInit() {
    // global_compressor.Start();
    // global_compressor.SetClosedLoopControl(true);

    // NOTE(Tyler): The PIDF tuning parameters are roughly tuned and could probably be improved
    global_left_shooter_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 30);
    global_left_shooter_motor.SetSensorPhase(true);
    global_left_shooter_motor.Config_kF(0, 0.03, 30);
    global_left_shooter_motor.Config_kP(0, 0.14541, 30);
    global_left_shooter_motor.Config_kI(0, 0.0005, 30);
    global_left_shooter_motor.Config_kD(0, 0.0, 30);
    global_left_shooter_motor.ConfigMaxIntegralAccumulator(0, 700, 30);

    global_right_shooter_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 30);
    global_right_shooter_motor.SetSensorPhase(true);
    global_right_shooter_motor.Config_kF(0, 0.03, 30);
    global_right_shooter_motor.Config_kP(0, 0.14541, 30);
    global_right_shooter_motor.Config_kI(0, 0.0005, 30);
    global_right_shooter_motor.Config_kD(0, 0.0, 30);
    global_left_shooter_motor.ConfigMaxIntegralAccumulator(0, 700, 30);

#if ENABLE_PID_TUNING
    // Setup network table stuff for tuning
    nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = instance.GetTable("tuning_table");
    global_shooter_f_gain = table->GetEntry("f_gain");
    global_shooter_p_gain = table->GetEntry("p_gain");
#endif
}

void Robot::TeleopPeriodic() {
    f32 power = global_gamepad.GetRawAxis(RIGHT_Y_AXIS);
    f32 yaw = global_gamepad.GetRawAxis(RIGHT_X_AXIS)*0.75;
    global_robot_drive.ArcadeDrive(power, yaw, true);

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

#if ENABLE_PID_TUNING
    global_left_shooter_motor.Config_kF(0, global_shooter_f_gain.GetDouble(0.0), 30);
    global_left_shooter_motor.Config_kP(0, global_shooter_p_gain.GetDouble(0.0), 30);

    global_right_shooter_motor.Config_kF(0, global_shooter_f_gain.GetDouble(0.0), 30);
    global_right_shooter_motor.Config_kP(0, global_shooter_p_gain.GetDouble(0.0), 30);
#endif

    f64 shooter_power = global_gamepad.GetRawAxis(RIGHT_TRIGGER);
    f64 encoder_units_per_rotation = 4096.0;
    f64 rpm_to_encoder_units_per_100ms = (encoder_units_per_rotation / 600.0);
    f64 velocity = (shooter_power * 4600) * rpm_to_encoder_units_per_100ms;
    global_left_shooter_motor.Set(ControlMode::Velocity, -velocity);
    global_right_shooter_motor.Set(ControlMode::Velocity, velocity);

    printf("Shooter velocity: %f %f\n", 
           global_left_shooter_motor.GetSelectedSensorVelocity()/rpm_to_encoder_units_per_100ms,
           global_right_shooter_motor.GetSelectedSensorVelocity()/rpm_to_encoder_units_per_100ms);
}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() { return(frc::StartRobot<Robot>()); }
#endif
