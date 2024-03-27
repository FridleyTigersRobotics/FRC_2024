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
#include <Drivetrain.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>
#include <Arm.h>
#include <Climber.h>
#include <networktables/NetworkTable.h>
#include <LimelightHelpers.h>
#include <cameraserver/CameraServer.h>




void Robot::RobotInit() {
  frc::CameraServer::StartAutomaticCapture();
  // Autonomous Chooser
  m_autoChooser.SetDefaultOption( kAutoNameDefault,         kAutoNameDefault );
  m_autoChooser.AddOption       ( kAutoDrive,               kAutoDrive );
  m_autoChooser.AddOption       ( kShootCenter,             kShootCenter );
  m_autoChooser.AddOption       ( kShootCenterPickupCenter, kShootCenterPickupCenter );
  m_autoChooser.AddOption       ( kShootLeftPickupLeft,     kShootLeftPickupLeft );
  m_autoChooser.AddOption       ( kShootRightPickupRight,   kShootRightPickupRight );
  m_autoChooser.AddOption       ( kCenterShootRun,          kCenterShootRun );

  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);

  m_LimeRotatePid.SetSetpoint( m_limeAngleOffset );

  #if 0
        frc::SmartDashboard::PutNumber( "Move_x",                  double{0} );
        frc::SmartDashboard::PutNumber( "Move_ArmEndPosition",     double{0} );
        frc::SmartDashboard::PutNumber( "Move_m_startXArmPosition",double{0} );
        frc::SmartDashboard::PutNumber( "Move_Goal",               double{0} );
        frc::SmartDashboard::PutNumber( "Move_Out",                double{0} );
        frc::SmartDashboard::PutNumber( "Move_Angle",              double{0} );
  #endif
}


 void Robot::TeleopInit() {
  m_Climber.initClimber();
  m_Arm.initArm();

  m_Drivetrain.m_imu.Reset();
  m_DriveTargetAngle = 0;

  m_AutoXdirPid.SetTolerance( kXyPosTolerance,  kXyVelTolerance );
  m_AutoXdirPid.Reset( 0.0_m );

#if 0
  frc::SmartDashboard::PutNumber("LimeVelocityMax", m_limeVelMax);
  frc::SmartDashboard::PutNumber("LimeAccelMax",    m_limeAccMax);
  frc::SmartDashboard::PutNumber("LimePValue",           m_limeP);
  frc::SmartDashboard::PutNumber("LimeMaxOut",            m_limeMaxOutput);
  frc::SmartDashboard::PutNumber("LimeOutput",           0);
  frc::SmartDashboard::PutNumber("LimeOutputUnclamped",           0);
  frc::SmartDashboard::PutNumber("RotPValue2",            m_RotP);
  frc::SmartDashboard::PutNumber("LimeMinOut",            m_limeMinOutput);
  frc::SmartDashboard::PutNumber("LimeMinThresh",            m_limeMinThresh);
  frc::SmartDashboard::PutNumber("limeAngleOffset",            m_limeAngleOffset);
#endif
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
  frc::SmartDashboard::PutBoolean( "Field relative?",      m_fieldRelative );


#if 0
  double limeVelMax = frc::SmartDashboard::GetNumber("LimeVelocityMax",  m_limeVelMax);
  double limeAccMax = frc::SmartDashboard::GetNumber("LimeAccelMax",     m_limeAccMax);
  double limeP = frc::SmartDashboard::GetNumber("LimePValue",            m_limeP);
  double limeMaxOutput = frc::SmartDashboard::GetNumber("LimeMaxOut",            m_limeMaxOutput);
  double limeMinOutput = frc::SmartDashboard::GetNumber("LimeMinOut",            m_limeMinOutput);
  double limeMinThresh = frc::SmartDashboard::GetNumber("LimeMinThresh",            m_limeMinThresh);
  
  double limeAngleOffset = frc::SmartDashboard::GetNumber("limeAngleOffset",            m_limeAngleOffset);
  
  if ( limeAngleOffset != m_limeAngleOffset )
  {
    m_limeAngleOffset = limeAngleOffset;
    m_LimeRotatePid.SetSetpoint( m_limeAngleOffset );
  }

    

  if ( limeMinOutput != m_limeMinOutput )
  {
    m_limeMinOutput = limeMinOutput;
  }

  if ( limeMinThresh != m_limeMinThresh )
  {
    m_limeMinThresh = limeMinThresh;
  }

  if ( limeMaxOutput != m_limeMaxOutput )
  {
    m_limeMaxOutput = limeMaxOutput;
  }


  if (  
    limeVelMax != m_limeVelMax ||
    limeAccMax != m_limeAccMax ||
    limeP != m_limeP
  )
  {
     m_limeVelMax = limeVelMax;     
     m_limeAccMax = limeAccMax; 
     m_limeP = limeP;  

    m_LimeRotatePid.SetP( m_limeP );
    //m_LimeRotatePid.SetConstraints( { units::radians_per_second_t{limeVelMax}, units::radians_per_second_squared_t{limeAccMax} } );
    m_LimeRotatePid.Reset();
    m_LimeRotatePid.SetSetpoint( m_limeAngleOffset );
  }

  double RotP = frc::SmartDashboard::GetNumber("RotPValue",            m_RotP);
  if (m_RotP != RotP)
  {
    m_RotP = RotP;
    m_DriveRotatePid.SetP( m_RotP );
  }
#endif

}




  void Robot::TeleopPeriodic() 
  { 
  if ( m_driveController.GetBackButtonPressed() )
  {
    m_fieldRelative = !m_fieldRelative;
  }

  if ( m_driveController.GetStartButtonPressed() )
  {
    m_Drivetrain.m_imu.ZeroYaw();
    m_DriveTargetAngle = 0;
    m_DriveRotatePid.Reset();
  }

#if 0
  if ( m_driveController.GetBButtonPressed() )
  {
    m_DriveTargetAngle += 90;
  }
  if ( m_driveController.GetXButtonPressed() )
  {
    m_DriveTargetAngle -= 90;
  }
#else

  double targetWrappedAngle  = 0;
  bool   angleChanged        = false;

  if(m_driveController.GetYButtonPressed())
  {
    targetWrappedAngle = 0;
    angleChanged = true;
  }

  if (m_driveController.GetBButtonPressed())
  {
    targetWrappedAngle = 90;
    angleChanged = true;
  }

  if (m_driveController.GetAButtonPressed())
  {
    targetWrappedAngle = 180;
    angleChanged = true;
  }

  if (m_driveController.GetXButtonPressed())
  {
    targetWrappedAngle = -90;
    angleChanged = true;
  }

  double y     = m_driveController.GetRightY();
  double x     = m_driveController.GetRightX();
  double mag   = sqrt( x * x + y * y );
  double angle = atan( x / y );
  
  frc::SmartDashboard::PutNumber( "controler_mag",   mag );
  frc::SmartDashboard::PutNumber( "controler_angle", angle );

  if( mag > 0.9 )
  {
    targetWrappedAngle = angle;
    angleChanged       = true;
  }

  if ( angleChanged )
  {
    double unwrappedRobotAngle = m_Drivetrain.m_imu.GetAngle();
    double wrappedRobotAngle   = m_Drivetrain.m_imu.GetYaw();
    double angleDelta = targetWrappedAngle - wrappedRobotAngle;
    if ( angleDelta > 180 )
    {
      angleDelta -= 360;
    }
    if ( angleDelta <= -180 )
    {
      angleDelta += 360;
    }

    m_DriveTargetAngle = unwrappedRobotAngle + angleDelta;
  }
#endif

  double TriggerRotateSpeedMult = 2.0;
  m_DriveTargetAngle += TriggerRotateSpeedMult * frc::ApplyDeadband( m_driveController.GetRightTriggerAxis(), 0.05 );
  m_DriveTargetAngle -= TriggerRotateSpeedMult * frc::ApplyDeadband( m_driveController.GetLeftTriggerAxis(),  0.05 );


  m_DriveRotatePid.SetSetpoint( m_DriveTargetAngle );

  double limeTx                = LimelightHelpers::getTX();
  double limeRotSpeedUnClamped = m_LimeRotatePid.Calculate( limeTx );
  double limeRotSpeed          = 0;

  // ID will be -1 if no targets are seen.
  if ( LimelightHelpers::getFiducialID() >= 0 )
  {
    limeRotSpeed = std::clamp( limeRotSpeedUnClamped, -m_limeMaxOutput, m_limeMaxOutput );
  }


  double driveRotSpeedUnClamped = -m_DriveRotatePid.Calculate( m_Drivetrain.m_imu.GetAngle() );
  double driveRotSpeed          = std::clamp( driveRotSpeedUnClamped, -0.5, 0.5 );

  frc::SmartDashboard::PutNumber( "limeTx",                 limeTx );
  frc::SmartDashboard::PutNumber( "limeRotSpeedUnClamped",  limeRotSpeedUnClamped);
  frc::SmartDashboard::PutNumber( "limeRotSpeed",           limeRotSpeed);
  frc::SmartDashboard::PutNumber( "driveRotSpeedUnClamped", driveRotSpeedUnClamped);
  frc::SmartDashboard::PutNumber( "driveRotSpeed",          driveRotSpeed);
  frc::SmartDashboard::PutNumber( "RobotAngle",             m_Drivetrain.m_imu.GetAngle());
  frc::SmartDashboard::PutNumber( "DriveTargetAngle",       m_DriveTargetAngle);


  double DriveDeadband = 0.1;
  double DriveY = frc::ApplyDeadband( -m_driveController.GetLeftY(), DriveDeadband );
  double DriveX = frc::ApplyDeadband( -m_driveController.GetLeftX(), DriveDeadband );

  double RotationSpeedRatio = 0;

  if ( m_driveController.GetRightBumper() )
  {
    RotationSpeedRatio = limeRotSpeed;
  }
  else
  {
    RotationSpeedRatio = driveRotSpeed;
  }


    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate( DriveX ) * Drivetrain::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate( DriveY ) * Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rotationSpeed = RotationSpeedRatio * Drivetrain::kMaxAngularSpeed;

    m_Drivetrain.SetSpeeds( xSpeed, ySpeed, rotationSpeed );





    // Codriver Controls
    if ( m_coController.GetBackButtonPressed() )
    {
      m_controlModeEndGame = !m_controlModeEndGame;
    }

    if ( m_controlModeEndGame )
    {
      m_Arm.SetArmPosition( m_Arm.HOLD_START_POSITION );

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
    else // if ( m_controlModeEndGame )
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

    } // else // if ( m_controlModeEndGame )


    // Update all subsystems
    m_Drivetrain.updateDrivetrain( GetPeriod(), m_fieldRelative );
    m_Arm.updateArm();
    m_Climber.updateClimber();
    m_Shooter.updateShooter();
    m_Intake.updateIntake();
  }



#ifndef RUNNING_FRC_TESTS
int main() {
 
  return frc::StartRobot<Robot>();

}
#endif
