/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>

struct Robot : public frc::TimedRobot {
    void RobotInit();
    void TeleopPeriodic();
    void AutonomousInit();
    void AutonomousPeriodic();
};

struct DriveTrain {
    TalonSRX left_master_motor;
    TalonSRX right_master_motor;
}