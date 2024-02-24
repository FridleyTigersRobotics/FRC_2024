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
#include <networktables/NetworkTable.h>
#include <LimelightHelpers.h>


void Robot::RobotInit() {
}

 void Robot::TestInit() {
 }

 void Robot::TeleopInit() {
  m_Climber.initClimber();
  m_Arm.initArm();

 }



 void Robot::TestPeriodic() {

}

void Robot::DisabledInit()
{
  m_Arm.disableArm();
}



void Robot::RobotPeriodic()
{
  m_Climber.UpdateRoll( m_Drivetrain.m_imu.GetRoll() );

  m_Arm.UpdateSmartDashboardData();
  m_Intake.UpdateSmartDashboardData();
  m_Climber.UpdateSmartDashboardData();
  m_Shooter.UpdateSmartDashboardData();

  frc::SmartDashboard::PutBoolean( "m_controlModeEndGame", m_controlModeEndGame );
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

    // Codriver Controls
    if ( m_coController.GetBackButtonPressed() )
    {
      m_controlModeEndGame = !m_controlModeEndGame;
    }

    if ( m_controlModeEndGame )
    {
      // Climber
      if( m_coController.GetRightBumper() )
      {
        m_Climber.ChangeClimberState( m_Climber.ClimberDown );
      }
      else if ( m_coController.GetLeftBumper() )
      {
        m_Climber.ChangeClimberState( m_Climber.ClimberUp );
      }
      else
      {
        m_Climber.ChangeClimberState( m_Climber.ClimberStop );
      }
    }
    else
    {
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
        if( !m_Arm.ArmHold() )
        {
          m_Arm.SetArmPosition( m_Arm.SPEAKER );
        }
      }

      // Intake
      if ( m_coController.GetLeftBumper() )
      {
        m_Intake.ChangeIntakeState( m_Intake.Intake_Outtaking );
      }
      else if( m_coController.GetRightBumper() )
      {
        m_Intake.ChangeIntakeState( m_Intake.Intake_Intaking );
      }
      else if( m_Arm.ArmReadyForGroundIntake() )
      {
        m_Intake.ChangeIntakeState( m_Intake.Intake_IntakingWithSensor );
      }
      else
      {
        m_Intake.ChangeIntakeState( m_Intake.Intake_Stopped );
      }

      // Shooter
      m_Shooter.changeShooterState( m_coController.GetRightTriggerAxis() > 0.2 );

    }


    m_Drivetrain.updateDrivetrain( GetPeriod() );
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

#if CLIMBER_MANUAL_CONTROL
    double speedL = 0;
    double speedR = 0;
    if ( m_controlModeEndGame )
    {
      if ( m_coController.GetXButton() )
      {
        if ( m_coController.GetLeftBumper() )
        {
          speedL = 1.0;
        }
        if ( m_coController.GetRightBumper() )
        {
          speedR = 1.0;
        }
      }
      else if ( m_coController.GetYButton() )
      {
        if ( m_coController.GetLeftBumper() )
        {
          speedL = -1.0;
        }
        if ( m_coController.GetRightBumper() )
        {
          speedR = -1.0;
        }
      }

      m_Climber.manualControl( speedL, speedR );
    }
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

    m_Drivetrain.SetSpeeds( xSpeed, ySpeed, rot );

  }


































 void Robot::AutonomousInit() {
    AutonomousStateInit();
    m_autoStateDone = false; 
    m_autoState     = 0;

    TeleopInit(); 
    m_autoTimer.Stop();
    m_autoTimer.Reset();
    m_autoTimer.Start();
    m_xDirPid.SetTolerance( kXyPosTolerance,  kXyVelTolerance );
    m_yDirPid.SetTolerance( kXyPosTolerance,  kXyVelTolerance );
    m_rotPid.SetTolerance(  kRotPosTolerance, kRotVelTolerance );

    m_xDirPid.Reset( 0.0_m );
    m_yDirPid.Reset( 0.0_m );
    m_rotPid.Reset( 0.0_rad );

 }

  void Robot::AutonomousPeriodic() {
    AutonomousStateUpdate();
    RunAutoSequence();

    m_Drivetrain.updateDrivetrain( GetPeriod() );
    m_Arm.updateArm();
    m_Climber.updateClimber();
    m_Shooter.updateShooter();
    m_Intake.updateIntake();
  }








void Robot::Drivetrain_Stop() {
   m_Drivetrain.SetSpeeds( 0.0_mps, 0.0_mps, 0.0_rad_per_s );
}







 void Robot::DriveForDistance( 
  units::meter_t              xDistance, 
  units::meter_t              yDistance, 
  units::radian_t             rotRadians, 
  units::meters_per_second_t  xSpeedMult, 
  units::meters_per_second_t  ySpeedMult, 
  units::radians_per_second_t rotSpeedMult,
  units::time::second_t       maxTime
)
{
  frc::Pose2d pose = m_Drivetrain.m_odometry.GetPose();

  m_xDirPid.SetConstraints({xSpeedMult,   m_xyMaxAccel});
  m_yDirPid.SetConstraints({ySpeedMult,   m_xyMaxAccel});
  m_rotPid.SetConstraints( {rotSpeedMult, m_rotMaxAccel});

  m_xDirPid.SetGoal( xDistance );
  m_yDirPid.SetGoal( yDistance );
  m_rotPid.SetGoal(  rotRadians );

  units::meters_per_second_t  xSpeed  { m_xDirPid.Calculate( pose.X() ) };
  units::meters_per_second_t  ySpeed  { m_yDirPid.Calculate( pose.Y() ) };
  units::radians_per_second_t rotSpeed{ m_rotPid.Calculate( pose.Rotation().Radians() ) };

  m_Drivetrain.SetSpeeds( xSpeed, ySpeed, rotSpeed );

  if ( ( m_xDirPid.AtGoal() &&  m_yDirPid.AtGoal() &&  m_rotPid.AtGoal() ) || 
       ( m_autoTimer.Get() > maxTime ) )
  {
    Drivetrain_Stop();
    m_autoStateDone = true;
  }
}


void Robot::MoveArmForPickup()
{
  m_Arm.SetArmPosition( m_Arm.GROUND_PICKUP );
  m_Intake.ChangeIntakeState( m_Intake.Intake_IntakingWithSensor );
  m_autoStateDone = true;
}

void Robot::MoveArmForShooting()
{
  m_Arm.SetArmPosition( m_Arm.SPEAKER );
  m_Intake.ChangeIntakeState( m_Intake.Intake_Stopped );
  m_autoStateDone = true;
}



void Robot::AimAndPrepShoot( units::second_t maxTime )
{
  m_Arm.SetArmPosition( m_Arm.SPEAKER );
  m_Intake.ChangeIntakeState( m_Intake.Intake_Stopped );
  m_Shooter.changeShooterState( true );

  if ( m_Arm.ArmReadyForShooting() ||
       m_autoTimer.Get() > maxTime )
  {
    m_autoStateDone = true;
  }
#if 0
  double tx = LimelightHelpers::getTX();
  double angleToTurnTo = tx / 180.0 * std::numbers::pi;

  DriveForDistance( 0.0_m, 0.0_m, angleToTurnTo );
  m_autoStateDone = true;
#endif
}


void Robot::Shoot( units::second_t maxTime )
{
  m_Intake.ChangeIntakeState( m_Intake.Intake_Outtaking );

  if ( m_autoTimer.Get() > maxTime )
  {
    m_Intake.ChangeIntakeState( m_Intake.Intake_Stopped );
    m_autoStateDone = true;
  }
}



void Robot::AutonomousStateInit()
{
  m_xDirPid.Reset(0.0_m);
  m_yDirPid.Reset(0.0_m);
  m_rotPid.Reset(0.0_rad);

  m_autoTimer.Stop();
  m_autoTimer.Reset();
  m_autoTimer.Start();
  m_initialPose = m_Drivetrain.m_odometry.GetPose();
  m_Drivetrain.m_odometry.ResetPosition(
    frc::Rotation2d{units::degree_t {m_Drivetrain.m_imu.GetYaw()}},
    {m_Drivetrain.m_frontLeft.GetPosition(), m_Drivetrain.m_frontRight.GetPosition(),
     m_Drivetrain.m_backLeft.GetPosition(),  m_Drivetrain.m_backRight.GetPosition()},
    frc::Pose2d{}
  );
  m_atRotateSetpointCount = 0;
}


void Robot::AutonomousStateUpdate()
{

}



void Robot::RunAutoSequence()
{
  if ( m_autoStateDone )
  {
    m_autoState++;
    m_autoStateDone = false;
    AutonomousStateInit();
  }

  if ( m_autoState < (*autoSequence).size() )
  {
    (*autoSequence)[m_autoState]();
  }
}




#ifndef RUNNING_FRC_TESTS
int main() {
 
  return frc::StartRobot<Robot>();

}
#endif
