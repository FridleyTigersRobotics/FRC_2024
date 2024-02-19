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




// Function from Kauailab Website:
//   https://pdocs.kauailabs.com/navx-mxp/examples/mxp-io-expansion/
int GetAnalogChannelFromPin( int io_pin_number ) {
    //static const int MAX_NAVX_MXP_DIGIO_PIN_NUMBER      = 9;
    static const int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER   = 3;
    //static const int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER  = 1;
    //static const int NUM_ROBORIO_ONBOARD_DIGIO_PINS     = 10;
    //static const int NUM_ROBORIO_ONBOARD_PWM_PINS       = 10;
    static const int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS  = 4;
    int roborio_channel = 0;

    if ( io_pin_number < 0 ) {
        throw std::runtime_error("Error:  navX-MXP I/O Pin #");
    }

    if ( io_pin_number > MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER ) {
        throw new std::runtime_error("Error:  Invalid navX-MXP Analog Input Pin #");
    }
    roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_ANALOGIN_PINS;

    return roborio_channel;
}

//#include <frc/PowerDistribution.h>


void Robot::RobotInit() {


}

 void Robot::TestInit() {
 }

 void Robot::TeleopInit() {

  m_Arm.initArm();


  //frc::PowerDistribution::ClearStickyFaults();
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


#if 0
    if ( m_coController.GetAButton() )
    {
      m_Climber.ChangeClimberState( ClimberUp );

    }
    else
    {
      if (m_coController.GetBButton())
      {
        m_Climber.ChangeClimberState( ClimberDown );

      }
      else
        {
          m_Climber.ChangeClimberState( ClimberStop );
        }
    }

    if (m_coController.GetXButton())
    {
      m_Arm.m_ArmMotorLeft.Set(1);
      m_Arm.m_ArmMotorRight.Set(1);
    }
    else
    {
      if (m_coController.GetYButton())
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


  if (m_coController.GetLeftBumper())
  {
    m_Arm.m_WristMotor.Set(1);
  }
  else
  {
    if (m_coController.GetRightBumper())
    {
      m_Arm.m_WristMotor.Set(-1);
    }
    else
    {
      m_Arm.m_WristMotor.Set(0);
    }
  }

  if (m_coController.GetAButton())
  {
    m_Shooter.m_shooterMotor.Set(1);
  }
  else
  {
    if(m_coController.GetBButton())
    {
    m_Shooter.m_shooterMotor.Set(-1);
    }
    else
    {
      m_Shooter.m_shooterMotor.Set(0);
    }
  }
#endif

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
    if( m_coController.GetBButtonPressed() )
    {
      m_Arm.SetArmPosition( m_Arm.GROUND_PICKUP );
    }
    else if( m_coController.GetAButtonPressed() )
    {
      m_Arm.SetArmPosition( m_Arm.SPEAKER );
    }
    else if( m_coController.GetXButtonPressed() )
    {
      m_Arm.SetArmPosition( m_Arm.AMP );
    }
    else if( m_coController.GetYButtonPressed() )
    {
      m_Arm.SetArmPosition( m_Arm.SOURCE );
    }

    // Intake
    if( m_coController.GetRightBumper() )
    {
      m_Intake.ChangeIntakeState( m_Intake.Intake_Intaking );
    }
    else if ( m_coController.GetLeftBumper() )
    {
      m_Intake.ChangeIntakeState( m_Intake.Intake_Outtaking );
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


#if 0
//Arm and intake code
if (m_coController.GetLeftBumperPressed())
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

if (m_coController.GetRightBumperPressed())
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

if (m_coController.GetAButton())
{
  m_Climber.m_ClimberState=ClimberUp;
  
}
else
  {
    if (m_coController.GetBButton())
    {
      m_Climber.m_ClimberState=ClimberDown;
      
    }
    else
      {
        m_Climber.m_ClimberState=ClimberStop;
      }
  }
#endif




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
