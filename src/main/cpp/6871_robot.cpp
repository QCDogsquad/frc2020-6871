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
TalonSRX global_left_master_drive = TalonSRX(4);
TalonSRX global_left_slave_drive = TalonSRX(3);
TalonSRX global_right_slave_drive = TalonSRX(1);
TalonSRX global_right_master_drive = TalonSRX(2);

TalonSRX global_left_shooter_motor = TalonSRX(7);
TalonSRX global_right_shooter_motor = TalonSRX(6);

Compressor global_compressor = Compressor();

nt::NetworkTableEntry global_shooter_f_gain;
nt::NetworkTableEntry global_shooter_p_gain;
nt::NetworkTableEntry global_shooter_i_gain;
nt::NetworkTableEntry global_shooter_d_gain;

void Robot::RobotInit() {
    // global_compressor.Start();
    // global_compressor.SetClosedLoopControl(true);

    // Configure shooter
    // -----------------------------------------------------------------------------------------------------
    global_left_shooter_motor.ConfigFactoryDefault();
    global_left_shooter_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 30);
    global_left_shooter_motor.SetSensorPhase(true);
    global_left_shooter_motor.SetNeutralMode(NeutralMode::Brake);
    global_left_shooter_motor.Config_kF(0, 0.0, 30);
    global_left_shooter_motor.Config_kP(0, 0.0, 30);
    global_left_shooter_motor.Config_kI(0, 0.0, 30);
    global_left_shooter_motor.Config_kD(0, 0.0, 30);
    global_left_shooter_motor.ConfigMaxIntegralAccumulator(0, 700, 30);

    global_right_shooter_motor.ConfigFactoryDefault();
    global_right_shooter_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 30);
    global_right_shooter_motor.SetSensorPhase(true);
    global_right_shooter_motor.SetNeutralMode(NeutralMode::Brake);
    global_right_shooter_motor.Config_kF(0, 0.0, 30);
    global_right_shooter_motor.Config_kP(0, 0.0, 30);
    global_right_shooter_motor.Config_kI(0, 0.0, 30);
    global_right_shooter_motor.Config_kD(0, 0.0, 30);
    //  global_right_shooter_motor.ConfigMaxIntegralAccumulator(0, 700, 30);


    // Configure drivetrain
    // -----------------------------------------------------------------------------------------------------
    global_left_slave_drive.Follow(global_left_master_drive);
    global_right_slave_drive.Follow(global_right_master_drive);
    global_left_master_drive.SetInverted(true);
    global_left_slave_drive.SetInverted(true);

    global_left_master_drive.SetNeutralMode(NeutralMode::Brake);
    global_left_slave_drive.SetNeutralMode(NeutralMode::Brake);
    global_right_master_drive.SetNeutralMode(NeutralMode::Brake);
    global_right_slave_drive.SetNeutralMode(NeutralMode::Brake);

    // NOTE(Tyler): First set of PIDF constants is for power, and the second set is for the turn
    global_left_master_drive.Config_kF(0, 0.0);
    global_left_master_drive.Config_kP(0, 0.1);
    global_left_master_drive.Config_kI(0, 0.0);
    global_left_master_drive.Config_kD(0, 0.0);
    // global_left_master_drive.ConfigMaxIntegralAccumulator(0, 700);

    global_right_master_drive.Config_kF(0, 0.0);
    global_right_master_drive.Config_kP(0, 0.1);
    global_right_master_drive.Config_kI(0, 0.0);
    global_right_master_drive.Config_kD(0, 0.0);
    // global_right_master_drive.ConfigMaxIntegralAccumulator(0, 700);

    nt::NetworkTableInstance table_instance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = table_instance.GetTable("tuning_table");
    global_shooter_f_gain = table->GetEntry("f_gain");
    global_shooter_p_gain = table->GetEntry("p_gain");
    global_shooter_i_gain = table->GetEntry("i_gain");
    global_shooter_d_gain = table->GetEntry("d_gain");
}

void Robot::TeleopPeriodic() {
    // Drive robot
    // -----------------------------------------------------------------------------------------------------
    {
        f64 power = global_gamepad.GetRawAxis(RIGHT_Y_AXIS);

        f64 yaw = global_gamepad.GetRawAxis(RIGHT_X_AXIS);

        global_left_master_drive.Set(ControlMode::PercentOutput, power, DemandType_ArbitraryFeedForward, -yaw);
        global_right_master_drive.Set(ControlMode::PercentOutput, power, DemandType_ArbitraryFeedForward, yaw);

        // printf("Drivetrain velocity: %f %f\n", 
        //        hi_res_cimcoder_units_to_rpm(global_left_master_drive.GetSelectedSensorVelocity(0)),
        //        hi_res_cimcoder_units_to_rpm(global_right_master_drive.GetSelectedSensorVelocity(0)));
    }

    // Control the shooter
    // -----------------------------------------------------------------------------------------------------
    {
        global_left_shooter_motor.Config_kF(0, global_shooter_f_gain.GetDouble(0.0), 30);
        global_left_shooter_motor.Config_kP(0, global_shooter_p_gain.GetDouble(0.0), 30);
        global_left_shooter_motor.Config_kI(0, global_shooter_i_gain.GetDouble(0.0), 30);
        global_left_shooter_motor.Config_kD(0, global_shooter_d_gain.GetDouble(0.0), 30);

        global_right_shooter_motor.Config_kF(0, global_shooter_f_gain.GetDouble(0.0), 30);
        global_right_shooter_motor.Config_kP(0, global_shooter_p_gain.GetDouble(0.0), 30);
        global_right_shooter_motor.Config_kI(0, global_shooter_i_gain.GetDouble(0.0), 30);
        global_right_shooter_motor.Config_kD(0, global_shooter_d_gain.GetDouble(0.0), 30);

        f64 shooter_power = global_gamepad.GetRawAxis(RIGHT_TRIGGER);
        f64 velocity = rpm_to_ctre_mag_units(shooter_power * 2800);
        global_left_shooter_motor.Set(ControlMode::Velocity, -velocity);
        global_right_shooter_motor.Set(ControlMode::Velocity, velocity);

        printf("Shooter velocity: %f %f\n", 
            ctre_mag_units_to_rpm(global_left_shooter_motor.GetSelectedSensorVelocity(0)),
            ctre_mag_units_to_rpm(global_right_shooter_motor.GetSelectedSensorVelocity(0)));
    }

    // Miscellaneous other controls
    // -----------------------------------------------------------------------------------------------------
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
}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() { return(frc::StartRobot<Robot>()); }
#endif
