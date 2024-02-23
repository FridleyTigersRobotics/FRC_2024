// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <Robot.h>
#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <fmt/printf.h>
#include <frc/filter/SlewRateLimiter.h>
#include <Shooter.h>
#include "Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>
#include <Arm.h>
#include <Climber.h>


void Robot::RobotInit() {


}

 void Robot::TestInit() {
 }

 void Robot::TeleopInit() {

  m_Arm.initArm();

 }


 void Robot::AutonomousInit() {
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

 void Robot::TestPeriodic() {

}

void Robot::DisabledInit()
{
  m_Arm.disableArm();
}



void Robot::RobotPeriodic()
{
  m_Arm.UpdateSmartDashboardData();
  m_Intake.UpdateSmartDashboardData();
  m_Climber.UpdateSmartDashboardData();
  m_Shooter.UpdateSmartDashboardData();
}









  void Robot::AutonomousPeriodic() {
    RunDriveAuto();
    m_swerve.UpdateOdometry();
  }



  void Robot::TeleopPeriodic() 
  { 
    // Driver Control
    DriveWithJoystick(
      false,
      -m_driveController.GetLeftY(),
      -m_driveController.GetLeftX(),
      m_driveController.GetRightX()
    ); 

    // Climber
    if( m_driveController.GetRightBumper() )
    {
      m_Climber.ChangeClimberState( m_Climber.ClimberDown );
    }
    else if ( m_driveController.GetLeftBumper() )
    {
      m_Climber.ChangeClimberState( m_Climber.ClimberUp );
    }
    else
    {
      m_Climber.ChangeClimberState( m_Climber.ClimberStop );
    }


    // Codriver Controls

    // Arm / Wrist
    if( m_coController.GetAButton() )
    {
      m_Arm.SetArmPosition( m_Arm.GROUND_PICKUP );
    }
    else if( m_coController.GetXButton() )
    {
      m_Arm.SetArmPosition( m_Arm.AMP );
    }
    else if( m_coController.GetYButton() )
    {
      m_Arm.SetArmPosition( m_Arm.SOURCE );
    }
    else
    {
      m_Arm.SetArmPosition( m_Arm.SPEAKER );
    }

    // Intake
    if ( m_coController.GetLeftBumper() )
    {
      m_Intake.ChangeIntakeState( m_Intake.Intake_Outtaking );
    }
    else if( m_coController.GetRightBumper() ||
        m_Arm.ArmReadyForGroundIntake() )
    {
      m_Intake.ChangeIntakeState( m_Intake.Intake_Intaking );
    }
    else
    {
      m_Intake.ChangeIntakeState( m_Intake.Intake_Stopped );
    }

    // Shooter
    m_Shooter.changeShooterState( m_coController.GetRightTriggerAxis() > 0.2 );


    m_Arm.updateArm();
    m_Climber.updateClimber();
    m_Shooter.updateShooter();
    m_Intake.updateIntake();

  #if DBG_MANUAL_CONTROL_ARM_MOTORS
    m_Arm.armManualControl( frc::ApplyDeadband(m_coController.GetLeftY(), 0.25 ) );
  #endif

  #if DBG_MANUAL_CONTROL_WRIST_MOTORS
    m_Arm.wristManualControl(frc::ApplyDeadband(m_coController.GetRightY(), 0.25 ));
  #endif

#if 0
    double speedL = 0;
    double speedR = 0;

    if ( m_driveController.GetXButton() )
    {
      if ( m_driveController.GetLeftBumper() )
      {
        speedL = 1.0;
      }
      if ( m_driveController.GetRightBumper() )
      {
        speedR = 1.0;
      }
    }
    else if ( m_driveController.GetYButton() )
    {
      if ( m_driveController.GetLeftBumper() )
      {
        speedL = -1.0;
      }
      if ( m_driveController.GetRightBumper() )
      {
        speedR = -1.0;
      }
    }

    m_Climber.manualControl( speedL, speedR );
#endif
  }




  void Robot::DriveWithJoystick(
    bool fieldRelative,
    double xSpeedInput,
    double ySpeedInput,
    double rotInput
  ) 
  {

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate(
                            frc::ApplyDeadband(xSpeedInput, 0.25)) *
                        Drivetrain::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate(
                            frc::ApplyDeadband(ySpeedInput, 0.25)) *
                        Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -m_rotLimiter.Calculate(
                         frc::ApplyDeadband(rotInput, 0.25)) *
                     Drivetrain::kMaxAngularSpeed;

    frc::SmartDashboard::PutNumber("m_driveController.GetLeftY",double{m_driveController.GetLeftY()});
    frc::SmartDashboard::PutNumber("m_driveController.GetLeftX()",double{m_driveController.GetLeftX()});
    frc::SmartDashboard::PutNumber("m_driveController.GetRightX()",double{m_driveController.GetRightX()});


    frc::SmartDashboard::PutNumber("Xspeed",double{xSpeed});
    frc::SmartDashboard::PutNumber("Yspeed",double{ySpeed});
    frc::SmartDashboard::PutNumber("Rot",double{rot});

    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, GetPeriod());

  }
































void Robot::Drivetrain_Drive(units::meters_per_second_t xSpeed,
                             units::radians_per_second_t rot) {
  m_swerve.Drive(xSpeed, 0.0_mps, rot, false, GetPeriod());
}


void Robot::Drivetrain_Stop() {
   m_swerve.Drive(0.0_mps, 0.0_mps, units::radians_per_second_t{0.0}, false, GetPeriod());
}


 bool Robot::DriveForDistance( units::meters_per_second_t speed, units::meter_t distance, units::time::second_t maxTime=5.0_s )
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


bool Robot::RunDriveAuto()
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
















#ifndef RUNNING_FRC_TESTS
int main() {
 
  return frc::StartRobot<Robot>();

}
#endif
