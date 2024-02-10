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
#include <Arm.h>
#include <Climber.h>
class Robot : public frc::TimedRobot {
 public:

 void AutonomousInit() override {
    TeleopInit(); 
    m_autoTimer.Stop();
    m_autoTimer.Reset();
    m_autoTimer.Start();
    

    m_autoSequence = 0;
    m_initState = true;
    m_autoState = 0; 
    m_initialAngle = 0;
   
    m_atRotateSetpointCount = 0;

 }

 void TestInit() override {

 
     if (m_controller.GetAButton())
  {
    m_Climber.m_ClimberState=ClimberUp;

  }
  else
    {
      if (m_controller.GetBButton())
      {
        m_Climber.m_ClimberState=ClimberDown;

      }
      else
        {
          m_Climber.m_ClimberState=ClimberStop;
        }
    }

  if (m_controller.GetXButton())
  {
    m_Arm.m_ArmMotorLeft.Set(1);
    m_Arm.m_ArmMotorRight.Set(1);
  }
  else
  {
    if (m_controller.GetYButton())
    {
      m_Arm.m_ArmMotorLeft.Set(-1);
      m_Arm.m_ArmMotorRight.Set(-1);
    }
    else
    {
      m_Arm.m_ArmMotorLeft.Set(0);
      m_Arm.m_ArmMotorRight.Set(0);
    }
  }


if (m_controller.GetLeftBumper())
{
  m_Arm.m_WristMotor.Set(1);
}
else
{
  if (m_controller.GetRightBumper())
  {
    m_Arm.m_WristMotor.Set(-1);
  }
  else
  {
    m_Arm.m_WristMotor.Set(0);
  }
}

if (m_secondcontroller.GetAButton())
{
  m_Shooter.m_ShooterMotor.Set(1);
}
else
{
  if(m_secondcontroller.GetBButton())
  {
  m_Shooter.m_ShooterMotor.Set(-1);
  }
  else
  {
    m_Shooter.m_ShooterMotor.Set(0);
  }
}


}


void Drivetrain_Drive(units::meters_per_second_t xSpeed,
                             units::radians_per_second_t rot) {
  m_swerve.Drive(xSpeed, 0.0_mps, rot, false, GetPeriod());
}


void Drivetrain_Stop() {
   m_swerve.Drive(0.0_mps, 0.0_mps, units::radians_per_second_t{0.0}, false, GetPeriod());
}


 bool DriveForDistance( units::meters_per_second_t speed, units::meter_t distance, units::time::second_t maxTime=5.0_s )
  {
    frc::Pose2d pose = m_swerve.m_odometry.GetPose();
    frc::SmartDashboard::PutNumber("pose.X()",double{pose.X()});
    frc::SmartDashboard::PutNumber("m_startDistance",double{m_startDistance});
    frc::SmartDashboard::PutNumber("distance",double{distance});
    if ( ( ( ( speed > 0.0_mps ) && ( (pose.X() - m_startDistance) < distance ) ) ||
           ( ( speed < 0.0_mps ) && ( (pose.X() - m_startDistance) > distance ) ) ) &&
         ( m_autoTimer.Get() < maxTime ) )
    {
      Drivetrain_Drive( speed, 0.0_rad_per_s );
      return false;
    }
    else
    {
      Drivetrain_Stop();
      return true;
    }
  }


  bool RunDriveAuto()
{
 frc::Pose2d pose = m_swerve.m_odometry.GetPose();
    bool sequenceDone = false;
    bool stateDone = false;

    switch( m_autoState )
    {
      case 0:
      {
        if ( m_initState )
        {
          m_autoTimer.Stop();
          m_autoTimer.Reset();
          m_autoTimer.Start();
          m_initialAngle = m_swerve.m_imu.GetAngle();
          m_startDistance = pose.X();
        }
        //SetLiftSetpoints( LIFT_POSITION_DRIVING );
        stateDone = DriveForDistance( 0.5_mps, 1.0_m, 6.0_s );
        break;
      }

      default:
      {
        Drivetrain_Stop();
        sequenceDone = true;

        // Done, do nothing
        break;
      }
    }

    if ( m_initState )
    {
      m_initState = false;
    }

    if ( stateDone )
    {
      //fmt::print( "stateDone {}\n", m_autoState );
      m_autoState++;
      m_initState = true;
    }

    return sequenceDone;
}














  void AutonomousPeriodic() override {
    RunDriveAuto();
    m_swerve.UpdateOdometry();
  }

  void TeleopPeriodic() override 
  { 
    DriveWithJoystick(false); 
     m_Arm.updateArm();
    m_Climber.updateClimber();
  }
 private:
  frc::XboxController m_controller{0};
  frc::XboxController m_secondcontroller{1};
  Drivetrain m_swerve;
  Arm m_Arm;
  Climber m_Climber;
  Shooter m_Shooter;
  //int m_Count=0;
 // std::string m_smart="idk";
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};
frc::Timer   m_autoTimer;
  //std::string  m_autoSelected { kAutoNameDefault };
  int          m_autoSequence { 0 };
  bool         m_initState    { true };
  unsigned int m_autoState    { 0 }; 
  double       m_initialAngle { 0 };
  double m_prevAngle             { 0 };
  double m_currentAngle          { 0 };
  double m_angleDelta            { 0 };
  double m_currentAvgAngle       { 0 };
  double m_avgAngleDelta         { 0 };
  double m_prevAvgAngle          { 0 };
  int m_atRotateSetpointCount    { 0 };
  units::meter_t m_startDistance { 0 };


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

//Arm and intake code
if (m_controller.GetLeftBumperPressed())
{
  if (m_Arm.m_ArmPosition==GROUND_PICKUP)
  {
    m_Arm.SetArmPosition(SOURCE);
  }
  else
  {
    if (m_Arm.m_ArmPosition==SOURCE)
    {
     m_Arm.SetArmPosition(SPEAKER);
    }
    else
    {
        if (m_Arm.m_ArmPosition==SPEAKER)
       {
          m_Arm.SetArmPosition(AMP);
       }
        else
        {
          if (m_Arm.m_ArmPosition==AMP)
          {
            m_Arm.SetArmPosition(TRAP);
          }
          else
          {
            if (m_Arm.m_ArmPosition==TRAP)
            {
               m_Arm.SetArmPosition(GROUND_PICKUP);
            }
            
          }
        }
    }
  }
}

if (m_controller.GetRightBumperPressed())
{
  if (m_Arm.m_ArmPosition==SOURCE)
  {
    m_Arm.SetArmPosition(GROUND_PICKUP);
  }
  else
  {
    if (m_Arm.m_ArmPosition==SPEAKER)
    {
     m_Arm.SetArmPosition(SOURCE);
    }
    else
    {
        if (m_Arm.m_ArmPosition==AMP)
       {
          m_Arm.SetArmPosition(SPEAKER);
       }
        else
        {
          if (m_Arm.m_ArmPosition==TRAP)
          {
            m_Arm.SetArmPosition(AMP);
          }
          else
          {
            if (m_Arm.m_ArmPosition==GROUND_PICKUP)
            {
               m_Arm.SetArmPosition(TRAP);
            }
            
          }
        }
    }
  }
}

if (m_controller.GetAButton())
{
  m_Climber.m_ClimberState=ClimberUp;
  
}
else
  {
    if (m_controller.GetBButton())
    {
      m_Climber.m_ClimberState=ClimberDown;
      
    }
    else
      {
        m_Climber.m_ClimberState=ClimberStop;
      }
  }





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
