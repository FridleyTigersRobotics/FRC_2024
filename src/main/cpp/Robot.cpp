// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <Shooter.h>
#include "Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>

class Robot : public frc::TimedRobot {
 public:
  void AutonomousPeriodic() override {
    DriveWithJoystick(false);
    m_swerve.UpdateOdometry();
  }

  void TeleopPeriodic() override 
  { 
    DriveWithJoystick(false); 
    //frc::SmartDashboard::PutNumber("Eli's brain power",m_Count);
   /* if (m_controller.GetAButton())
    {
      m_Count = m_Count * 5;
    }
    if (m_controller.GetBButton())
    {
      m_Count=0;
    }
    if (m_controller.GetXButton())
    {
      m_Count++;
    }
    frc::SmartDashboard::PutString("Is Eli super smart??",m_smart);
    if (m_controller.GetLeftBumperPressed())
    {
      m_smart="yes";
    }
   if (m_controller.GetRightBumperPressed())
    {
      m_smart="no";
    }
 if (m_controller.GetYButton())
    {
      m_smart="idk";
    }*/
  }
 private:
  frc::XboxController m_controller{0};
  Drivetrain m_swerve;
  //int m_Count=0;
 // std::string m_smart="idk";
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};



  void DriveWithJoystick(bool fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_controller.GetLeftY(), 0.25)) *
                        Drivetrain::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_controller.GetLeftX(), 0.25)) *
                        Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -m_rotLimiter.Calculate(
                         frc::ApplyDeadband(m_controller.GetRightX(), 0.25)) *
                     Drivetrain::kMaxAngularSpeed;

frc::SmartDashboard::PutNumber("m_controller.GetLeftY",double{m_controller.GetLeftY()});
frc::SmartDashboard::PutNumber("m_controller.GetLeftX()",double{m_controller.GetLeftX()});
frc::SmartDashboard::PutNumber("m_controller.GetRightX()",double{m_controller.GetRightX()});


frc::SmartDashboard::PutNumber("Xspeed",double{xSpeed});
frc::SmartDashboard::PutNumber("Yspeed",double{ySpeed});
frc::SmartDashboard::PutNumber("Rot",double{rot});

    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, GetPeriod());
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
