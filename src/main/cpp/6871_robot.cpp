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
#include <frc/DigitalInput.h>
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


Compressor global_compressor = Compressor();

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

nt::NetworkTableEntry global_shooter_f_gain;
nt::NetworkTableEntry global_shooter_p_gain;
nt::NetworkTableEntry global_shooter_i_gain;
nt::NetworkTableEntry global_shooter_d_gain;

VictorSPX global_control_panel_spinner = VictorSPX(2);

VictorSPX global_winch_motor = VictorSPX(4);
DoubleSolenoid global_lift_pistons = DoubleSolenoid(2, 1);
DigitalInput global_lift_limit_switch = DigitalInput(0);
b32 global_lift_state;

void Robot::RobotInit() {
    global_compressor.Start();
    global_compressor.SetClosedLoopControl(true);

    // Configure shooter
    // -----------------------------------------------------------------------------------------------------
    TalonSRXConfiguration shooter_config = TalonSRXConfiguration();
    shooter_config.slot0.kF = 0.0;
    shooter_config.slot0.kP = 0.1;
    shooter_config.slot0.kI = 0.0;
    shooter_config.slot0.kD = 0.0;
    shooter_config.slot0.maxIntegralAccumulator = 700;
    shooter_config.primaryPID.selectedFeedbackSensor = FeedbackDevice::CTRE_MagEncoder_Relative;

    global_left_shooter_motor.SetSensorPhase(true);
    global_left_shooter_motor.SetNeutralMode(NeutralMode::Brake);
    global_left_shooter_motor.ConfigAllSettings(shooter_config);

    global_right_shooter_motor.SetSensorPhase(true);
    global_right_shooter_motor.SetNeutralMode(NeutralMode::Brake);
    global_right_shooter_motor.ConfigAllSettings(shooter_config);

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

    // TODO(Tyler): Tune the PIDF constants
    TalonSRXConfiguration drive_config = TalonSRXConfiguration();
    drive_config.slot0.kF = 0.0;
    drive_config.slot0.kP = 0.1;
    drive_config.slot0.kI = 0.0;
    drive_config.slot0.kD = 0.0;
    shooter_config.primaryPID.selectedFeedbackSensor = FeedbackDevice::CTRE_MagEncoder_Relative;
    global_left_master_drive.ConfigAllSettings(drive_config);
    global_right_master_drive.ConfigAllSettings(drive_config);

    nt::NetworkTableInstance table_instance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = table_instance.GetTable("tuning_table");
    char *prefix = "shooter";
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%s_f_gain");
    global_shooter_f_gain = table->GetEntry("f_gain");
    global_shooter_p_gain = table->GetEntry("p_gain");
    global_shooter_i_gain = table->GetEntry("i_gain");
    global_shooter_d_gain = table->GetEntry("d_gain");
}

inline void set_shooter_percent_of_max_rpm(f64 power) {
    f64 velocity = rpm_to_ctre_mag_units(power * 2800);
    global_left_shooter_motor.Set(ControlMode::Velocity, -velocity);
    global_right_shooter_motor.Set(ControlMode::Velocity, velocity);
}

inline f64 
get_average_shooter_speed() {
    f64 result = (global_left_shooter_motor.GetSelectedSensorVelocity(0) + global_right_shooter_motor.GetSelectedSensorVelocity(0)) / 2.0;
    return(result);
}

inline void 
set_motor_power_and_direction_by_buttons(BaseMotorController *motor, f64 power, 
                                              u32 forward_button, u32 reverse_button) {
    if(global_gamepad.GetRawButton(forward_button)) {
        motor->Set(ControlMode::PercentOutput, power);
    } else if(global_gamepad.GetRawButton(reverse_button)) {
        motor->Set(ControlMode::PercentOutput, -power);
    } else {
        motor->Set(ControlMode::PercentOutput, 0.0);
    }
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
        set_shooter_percent_of_max_rpm(shooter_power);

        printf("Shooter velocity: %f %f\n", 
            ctre_mag_units_to_rpm(global_left_shooter_motor.GetSelectedSensorVelocity(0)),
            ctre_mag_units_to_rpm(global_right_shooter_motor.GetSelectedSensorVelocity(0)));
    }

    // Miscellaneous other controls
    // -----------------------------------------------------------------------------------------------------
    if(global_gamepad.GetRawButtonPressed(B_BUTTON)) {
        global_is_intake_toggled = !global_is_intake_toggled;
    }

    if(global_is_intake_toggled) {
        global_intake_motor.Set(ControlMode::PercentOutput, -0.75);
    } else {
        global_intake_motor.Set(ControlMode::PercentOutput, 0.0);
    }
    
    if(global_gamepad.GetRawButtonPressed(Y_BUTTON)) {
        if(global_is_intake_raised) {
            global_intake_solenoid.Set(DoubleSolenoid::kForward);
        } else {
            global_intake_solenoid.Set(DoubleSolenoid::kReverse);
        }
        global_is_intake_raised = !global_is_intake_raised;
    }

    set_motor_power_and_direction_by_buttons(&global_uptake_motor, 1.0, 
                                             A_BUTTON, B_BUTTON);

    if(global_gamepad.GetPOV() == POV_LEFT) {
        global_control_panel_spinner.Set(ControlMode::PercentOutput, 0.5);
    } else if(global_gamepad.GetPOV() == POV_RIGHT) {
        global_control_panel_spinner.Set(ControlMode::PercentOutput, -0.5);
    } else {
        global_control_panel_spinner.Set(ControlMode::PercentOutput, 0.0);
    }


    if(global_gamepad.GetRawButtonPressed(BACK_BUTTON)) {
        global_lift_state = !global_lift_state;
        if(global_lift_state) {
            global_lift_pistons.Set(DoubleSolenoid::kForward);
        } else {
            global_lift_pistons.Set(DoubleSolenoid::kReverse);
        }
    }
    if(!global_lift_state && !global_lift_limit_switch.Get()) {
        global_winch_motor.Set(ControlMode::PercentOutput, 1.0);
    } else if(global_gamepad.GetRawButton(START_BUTTON)) {
        global_winch_motor.Set(ControlMode::PercentOutput, -1.0);
    } else {
        global_winch_motor.Set(ControlMode::PercentOutput, 0.0);
    }

    printf("lift state: %u \n", global_lift_state);

    printf("Limit switch: %u\n", global_lift_limit_switch.Get());
}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() { return(frc::StartRobot<Robot>()); }
#endif
