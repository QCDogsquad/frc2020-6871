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

#include "6871_robot.h"

#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <ctre/Phoenix.h>

#define rpm_to_ctre_mag_units(rpm) ((rpm)*(4096.0/600.0))
#define ctre_mag_units_to_rpm(units) ((units)/(4096.0/600.0))
#define rpm_to_hi_res_cimcoder_units(rpm) ((rpm)*(1024.0/600.0))
#define hi_res_cimcoder_units_to_rpm(units) ((units)/(1024.0/600.0))
#define square(value) ((value)*(value))

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
// TODO(Tyler): Perhaps rename these to master_[SIDE]_drive and slave_[SIDE]_drive
TalonSRX global_front_left_drive = TalonSRX(4);
TalonSRX global_rear_left_drive = TalonSRX(3);
TalonSRX global_front_right_drive = TalonSRX(1);
TalonSRX global_rear_right_drive = TalonSRX(2);

TalonSRX global_left_shooter_motor = TalonSRX(7);
TalonSRX global_right_shooter_motor = TalonSRX(6);

Compressor global_compressor = Compressor();

void Robot::RobotInit() {
    // global_compressor.Start();
    // global_compressor.SetClosedLoopControl(true);

    // Configure talons
    // -----------------------------------------------------------------------------------------------------
    // NOTE(Tyler): The PIDF tuning parameters are roughly tuned and could probably be improved
    global_left_shooter_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);
    global_left_shooter_motor.SetSensorPhase(true);
    global_left_shooter_motor.Config_kF(0, 0.03, 30);
    global_left_shooter_motor.Config_kP(0, 0.14541, 30);
    global_left_shooter_motor.Config_kI(0, 0.0005, 30);
    global_left_shooter_motor.Config_kD(0, 0.0, 30);
    global_left_shooter_motor.ConfigMaxIntegralAccumulator(0, 700, 30);

    global_right_shooter_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);
    global_right_shooter_motor.SetSensorPhase(true);
    global_right_shooter_motor.Config_kF(0, 0.03, 30);
    global_right_shooter_motor.Config_kP(0, 0.14541, 30);
    global_right_shooter_motor.Config_kI(0, 0.0005, 30);
    global_right_shooter_motor.Config_kD(0, 0.0, 30);
    global_left_shooter_motor.ConfigMaxIntegralAccumulator(0, 700, 30);

    // Configure drivetrain
    // -----------------------------------------------------------------------------------------------------
    // TODO(Tyler): This might need more setup! like changing PID0 - PID1 to PID0 + PID1 for auxillary PID control
    global_rear_left_drive.Follow(global_front_left_drive);
    global_front_right_drive.Follow(global_rear_right_drive);
    // global_front_right_drive.SetInverted(true);
    // global_rear_right_drive.SetInverted(true);
    // global_front_left_drive.SetSensorPhase(true);
    // global_rear_right_drive.SetSensorPhase(true);

    global_front_left_drive.SetNeutralMode(NeutralMode::Coast);
    global_rear_left_drive.SetNeutralMode(NeutralMode::Coast);
    global_front_right_drive.SetNeutralMode(NeutralMode::Coast);
    global_rear_right_drive.SetNeutralMode(NeutralMode::Coast);

    global_front_left_drive.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0);
    global_front_left_drive.ConfigRemoteFeedbackFilter(global_rear_right_drive.GetDeviceID(), 
                                                       RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor,
                                                       0);
    global_front_left_drive.ConfigSensorTerm(SensorTerm::SensorTerm_Diff1, FeedbackDevice::RemoteSensor0);
    global_front_left_drive.ConfigSensorTerm(SensorTerm::SensorTerm_Diff0, FeedbackDevice::QuadEncoder);
    global_front_left_drive.ConfigSelectedFeedbackSensor(FeedbackDevice::SensorDifference, 1);
    
    // NOTE(Tyler): First set of PIDF constants is for power, and the second set is for the turn
    global_front_left_drive.Config_kF(0, 0.15);
    global_front_left_drive.Config_kP(0, 0.05);
    global_front_left_drive.Config_kI(0, 0.001);
    global_front_left_drive.Config_kD(0, 0.0);
    global_front_left_drive.ConfigMaxIntegralAccumulator(0, 700);

    global_front_left_drive.Config_kF(1, 0.15);
    global_front_left_drive.Config_kP(1, 0.05);
    global_front_left_drive.Config_kI(1, 0.001);
    global_front_left_drive.Config_kD(1, 0.0);
    global_front_left_drive.ConfigMaxIntegralAccumulator(1, 700);

    // global_front_left_drive.ConfigAuxPIDPolarity(true);

    // global_rear_right_drive.Config_kF(0, 0.0);
    // global_rear_right_drive.Config_kP(0, 0.05);
    // global_rear_right_drive.Config_kI(0, 0.0);
    // global_rear_right_drive.Config_kD(0, 0.0);
    
    // global_rear_right_drive.Config_kF(1, 0.0);
    // global_rear_right_drive.Config_kP(1, 0.05);
    // global_rear_right_drive.Config_kI(1, 0.0);
    // global_rear_right_drive.Config_kD(1, 0.0);
}

void Robot::TeleopPeriodic() {
    // Drive robot
    // -----------------------------------------------------------------------------------------------------
    {
        f64 power = global_gamepad.GetRawAxis(RIGHT_Y_AXIS);
        f64 forward_velocity = rpm_to_hi_res_cimcoder_units(1000*square(power));

        f64 yaw = global_gamepad.GetRawAxis(RIGHT_X_AXIS);
        f64 turn_velocity = rpm_to_hi_res_cimcoder_units(1000*square(yaw));

        global_front_left_drive.Set(ControlMode::Velocity, power*forward_velocity, DemandType_AuxPID, turn_velocity);
        global_rear_right_drive.Follow(global_front_left_drive, FollowerType_AuxOutput1);

        printf("Drivetrain velocity: %f %f\n", 
               hi_res_cimcoder_units_to_rpm(global_rear_right_drive.GetSelectedSensorVelocity(0)),
               hi_res_cimcoder_units_to_rpm(global_rear_right_drive.GetSelectedSensorVelocity(1)));
    }

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


    // Control the shooter
    // -----------------------------------------------------------------------------------------------------
    {

        f64 shooter_power = global_gamepad.GetRawAxis(RIGHT_TRIGGER);
        f64 velocity = rpm_to_ctre_mag_units(shooter_power * 4600);
        global_left_shooter_motor.Set(ControlMode::Velocity, -velocity);
        global_right_shooter_motor.Set(ControlMode::Velocity, velocity);

        // printf("Shooter velocity: %f %f\n", 
        //     global_left_shooter_motor.GetSelectedSensorVelocity()/rpm_to_encoder_units_per_100ms,
        //     global_right_shooter_motor.GetSelectedSensorVelocity()/rpm_to_encoder_units_per_100ms);
    }
}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() { return(frc::StartRobot<Robot>()); }
#endif
