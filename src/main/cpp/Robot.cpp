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

    // Autonomous Chooser
    m_autoChooser.SetDefaultOption( kAutoNameDefault,  kAutoNameDefault );
    m_autoChooser.AddOption       ( kAutoDrive,        kAutoDrive );
    m_autoChooser.AddOption       ( kAutoShoot,        kAutoShoot );

    frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);



  #if 0
        frc::SmartDashboard::PutNumber( "Move_x",                  double{0} );
        frc::SmartDashboard::PutNumber( "Move_ArmEndPosition",     double{0} );
        frc::SmartDashboard::PutNumber( "Move_m_startXArmPosition",double{0} );
        frc::SmartDashboard::PutNumber( "Move_Goal",               double{0} );
        frc::SmartDashboard::PutNumber( "Move_Out",                double{0} );
        frc::SmartDashboard::PutNumber( "Move_Angle",              double{0} );
  #endif

}

 void Robot::TestInit() {
  m_Arm.disableArm();

 }

 void Robot::TeleopInit() {
  m_Climber.initClimber();
  m_Arm.initArm();

  //m_xDirPid.SetConstraints({100.0_mps, units::meters_per_second_squared_t{100.0}});
  m_xDirPid.SetTolerance( kXyPosTolerance,  kXyVelTolerance );
  m_xDirPid.Reset( 0.0_m );
  m_xDirPid2.Reset();
 }



void Robot::TestPeriodic() {
  m_Drivetrain.SetSpeeds( 0.0_mps, 0.0_mps, 0.0_rad_per_s );
}

void Robot::DisabledInit()
{

}



void Robot::RobotPeriodic()
{
  m_Climber.UpdateRoll( m_Drivetrain.m_imu.GetRoll() );

  m_Arm.UpdateSmartDashboardData();
  m_Intake.UpdateSmartDashboardData();
  m_Climber.UpdateSmartDashboardData();
  m_Shooter.UpdateSmartDashboardData();
  m_Drivetrain.UpdateSmartDashboardData();

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

    // if ( m_driveController.GetAButtonPressed() )
    // {
    //   m_Drivetrain.m_odometry.ResetPosition(
    //     frc::Rotation2d{units::degree_t {m_Drivetrain.m_imu.GetYaw()}},
    //     {m_Drivetrain.m_frontLeft.GetPosition(), m_Drivetrain.m_frontRight.GetPosition(),
    //     m_Drivetrain.m_backLeft.GetPosition(),  m_Drivetrain.m_backRight.GetPosition()},
    //     frc::Pose2d{}
    //   );
    // }
    


    // Codriver Controls
    if ( m_coController.GetBackButtonPressed() )
    {
      m_controlModeEndGame = !m_controlModeEndGame;
    }

    if ( m_controlModeEndGame )
    {
      if( m_coController.GetAButtonPressed() )
      {
        m_Drivetrain.m_odometry.ResetPosition(
          frc::Rotation2d{units::degree_t {m_Drivetrain.m_imu.GetYaw()}},
          {m_Drivetrain.m_frontLeft.GetPosition(), m_Drivetrain.m_frontRight.GetPosition(),
          m_Drivetrain.m_backLeft.GetPosition(),  m_Drivetrain.m_backRight.GetPosition()},
          frc::Pose2d{}
        );
        m_xDirPid.Reset( 0.0_m );
        m_xDirPid2.Reset();
        m_startXArmPosition = m_Arm.ArmEndPosition();
      }

      if( m_coController.GetAButton() )
      {
        frc::Pose2d pose = m_Drivetrain.m_odometry.GetPose();
        m_Arm.SetArmPosition( m_Arm.TRAP );
        double setpoint =  double{m_startXArmPosition - m_Arm.ArmEndPosition()};
        m_xDirPid2.SetSetpoint( setpoint );

        double  xSpeed{ m_xDirPid2.Calculate( double{pose.X()} ) };

      #if 0
        frc::SmartDashboard::PutNumber( "Move_x",                  double{pose.X()} );
        frc::SmartDashboard::PutNumber( "Move_ArmEndPosition",     double{m_Arm.ArmEndPosition()} );
        frc::SmartDashboard::PutNumber( "Move_m_startXArmPosition",double{m_startXArmPosition} );
        frc::SmartDashboard::PutNumber( "Move_Goal",               double{setpoint} );
        frc::SmartDashboard::PutNumber( "Move_Out",                double{xSpeed} );
      #endif

        m_Drivetrain.AddToSpeeds( units::meters_per_second_t{xSpeed}, 0.0_mps, 0.0_rad_per_s );

        // if ( m_Arm.ArmReadyForMoveForwardPreClimb() )
        // {
        //   m_Drivetrain.AddToSpeeds( -0.1_mps, 0.0_mps, 0.0_rad_per_s );
        // }
      }
      else
      {
        m_Arm.SetArmPosition( m_Arm.HOLD_START_POSITION );
      }

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

  #if 0
    frc::SmartDashboard::PutNumber("m_driveController.GetLeftY",double{m_driveController.GetLeftY()});
    frc::SmartDashboard::PutNumber("m_driveController.GetLeftX()",double{m_driveController.GetLeftX()});
    frc::SmartDashboard::PutNumber("m_driveController.GetRightX()",double{m_driveController.GetRightX()});


    frc::SmartDashboard::PutNumber("Xspeed",double{xSpeed});
    frc::SmartDashboard::PutNumber("Yspeed",double{ySpeed});
    frc::SmartDashboard::PutNumber("Rot",double{rot});
  #endif
    m_Drivetrain.SetSpeeds( xSpeed, ySpeed, rot );

  }


































 void Robot::AutonomousInit() {
    m_autoSelected = m_autoChooser.GetSelected();
    fmt::print("Auto selected: {}\n", m_autoSelected);


    if (m_autoSelected == kAutoDrive) 
    {
      autoSequence = &DriveAuto;
    }
    else if (m_autoSelected == kAutoShoot) 
    {
      autoSequence = &ShootAuto;
    }


    AutonomousStateInit();
    m_autoStateDone = false; 
    m_autoState     = 0;

    Drivetrain_Stop();
    m_Arm.SetArmPosition( m_Arm.HOLD_START_POSITION );
    m_Intake.ChangeIntakeState( m_Intake.Intake_Stopped );
    m_Shooter.changeShooterState( false );
    m_Climber.ChangeClimberState( m_Climber.ClimberStop );
    

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
  units::radians_per_second_t rotSpeed{ -m_rotPid.Calculate( pose.Rotation().Radians() ) };

  // fmt::printf( "dbg: %1d, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n", 
  //   m_autoState, 
  //   float{pose.X() },
  //   float{pose.Y()},
  //   float{pose.Rotation().Radians()},
  //   float{xSpeed},
  //   float{ySpeed},
  //   float{rotSpeed}
  // );


  /*frc::SmartDashboard::PutNumber( "Auto_xSpeed",   xSpeed );
  frc::SmartDashboard::PutNumber( "Auto_ySpeed",   ySpeed );
  frc::SmartDashboard::PutNumber( "Auto_rotSpeed", rotSpeed );*/


  m_Drivetrain.SetSpeeds( xSpeed, ySpeed, rotSpeed );
  //Drivetrain_Stop();


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

  if ( ( m_Arm.ArmReadyForShooting() && m_Shooter.shooterReadyToShoot() ) ||
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


void Robot::Wait( units::second_t maxTime )
{
  if ( m_autoTimer.Get() > maxTime )
  {
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
  //m_initialPose = m_Drivetrain.m_odometry.GetPose();
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

  // frc::SmartDashboard::PutNumber("Auto_Idx",  m_autoState);
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
