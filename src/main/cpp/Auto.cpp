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


#if NEW_AUTO_CHOOSER
std::vector<std::string> AutoModeNames = {
    "DO NOTHING",
    "Drive",
    "ShootCenter",
    "ShootCenterPickupCenter",
    "ShootLeftPickupLeft",
    "ShootRightPickupRight",
    "CenterShootRun",
    "ShootRightPickupRightTest",
    "ShootLeftPickupLeftTest",
    "ShootRunLeft"
};

std::string Robot::TranslateAutoModeToAutoString( uint32_t autoModeInt ) {
    std::string autoString = kAutoNameDefault;
    
    if ( autoModeInt < AutoModeNames.size() )
    {
      autoString = AutoModeNames[autoModeInt];
    }

    return autoString;
}

#endif

 void Robot::AutonomousInit() {
   
  #if NEW_AUTO_CHOOSER
    m_autoSelectedInteger = frc::SmartDashboard::GetNumber("AutoModeInt", 0 );
    fmt::print("Auto selected integer: {}\n", m_autoSelectedInteger);
    m_autoSelected = TranslateAutoModeToAutoString( m_autoSelectedInteger );
  #else
    m_autoSelected = m_autoChooser.GetSelected();
  #endif

    fmt::print("Auto selected: {}\n", m_autoSelected);

    if (m_autoSelected == kAutoDrive) 
    {
      autoSequence = &Auto_Drive;
    }
    else if (m_autoSelected == kShootCenter) 
    {
      autoSequence = &Auto_ShootCenter;
    }
    else if (m_autoSelected == kShootCenterPickupCenter) 
    {
      autoSequence = &Auto_ShootCenterPickupCenter;
    }
     else if (m_autoSelected == kShootLeftPickupLeft) 
    {
      autoSequence = &ShootLeftPickupLeft;
    }
     else if (m_autoSelected == kShootRightPickupRight) 
    {
      autoSequence = &ShootRightPickupRight;
    }
     else if (m_autoSelected == kCenterShootRun) 
    {
      autoSequence = &Auto_CenterShootRun;
    }
    else if (m_autoSelected == kShootRightPickupRightTest) 
    {
      autoSequence = &ShootRightPickupRightTest;
    }
    else if (m_autoSelected == kShootLeftPickupLeftTest) 
    {
      autoSequence = &ShootLeftPickupLeftTest;
    }
     else if (m_autoSelected == kShootRunLeft) 
    {
      autoSequence = &Auto_ShootRunLeft;
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
    m_fieldRelative = false;
    m_autoTimer.Stop();
    m_autoTimer.Reset();
    m_autoTimer.Start();
    m_AutoXdirPid.SetTolerance( kXyPosTolerance,  kXyVelTolerance );
    m_AutoYdirPid.SetTolerance( kXyPosTolerance,  kXyVelTolerance );
    m_AutoRotatePid.SetTolerance(  kRotPosTolerance, kRotVelTolerance );

    m_AutoXdirPid.Reset( 0.0_m );
    m_AutoYdirPid.Reset( 0.0_m );
    m_AutoRotatePid.Reset( 0.0_rad );

 }

  void Robot::AutonomousPeriodic() {
    AutonomousStateUpdate();
    RunAutoSequence();
    
    m_Drivetrain.updateDrivetrain( GetPeriod(), m_fieldRelative );
    m_Arm.updateArm();
    m_Climber.updateClimber();
    m_Shooter.updateShooter();
    m_Intake.updateIntake();
    
  }





void Robot::AutoAngle(double AngleCrap) {
  m_Drivetrain.m_YawOffset    = -AngleCrap;
  m_DriveTargetAngle          = -AngleCrap;
  m_autoStateDone = true;


  //Goal(facing away)=the current angle (if right (-1*120) jf left (+120))
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

  m_AutoXdirPid.SetConstraints({xSpeedMult,   m_xyMaxAccel});
  m_AutoYdirPid.SetConstraints({ySpeedMult,   m_xyMaxAccel});
  m_AutoRotatePid.SetConstraints( {rotSpeedMult, m_rotMaxAccel});

  m_AutoXdirPid.SetGoal( xDistance );
  m_AutoYdirPid.SetGoal( yDistance );
  m_AutoRotatePid.SetGoal( rotRadians );
  m_LimeRotatePid.SetSetpoint( m_limeAngleOffset );

  units::meters_per_second_t  xSpeed  { m_AutoXdirPid.Calculate( pose.X() ) };
  units::meters_per_second_t  ySpeed  { m_AutoYdirPid.Calculate( pose.Y() ) };


  units::radians_per_second_t rotSpeed{ -m_AutoRotatePid.Calculate( pose.Rotation().Radians() - m_initialPose.Rotation().Radians() ) };
 

  frc::SmartDashboard::PutNumber("Auto_angle", double{pose.Rotation().Radians()});
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


  if ( ( m_AutoXdirPid.AtGoal() &&  m_AutoYdirPid.AtGoal() &&  m_AutoRotatePid.AtGoal() ) || 
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

  if ( m_autoTimer.Get() > 1.0_s )
  {
    m_Intake.ChangeIntakeState( m_Intake.Intake_Stopped );
    m_autoStateDone = true;
  }
}



void Robot::AimAndPrepShoot( units::second_t maxTime )
{
  m_Arm.SetArmPosition( m_Arm.SPEAKER );
  //Drive for distance using the limlight aiming. We can use the same thing as we do in tele-op?
  //Yoink the variables from teleop in robot? hmm...
  m_Intake.ChangeIntakeState( m_Intake.Intake_Stopped );
  m_Shooter.changeShooterState( true );

  if ( ( m_Arm.ArmReadyForShooting() && m_Shooter.shooterReadyToShoot() ) ||
       m_autoTimer.Get() > maxTime )
  {
    m_autoStateDone = true;
  }
#if 0

  double angleToTurnTo = tx / 180.0 * (std::numbers::pi*2);

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
  m_AutoXdirPid.Reset(0.0_m);
  m_AutoYdirPid.Reset(0.0_m);
  m_AutoRotatePid.Reset(0.0_rad);

  m_autoTimer.Stop();
  m_autoTimer.Reset();
  m_autoTimer.Start();
  
  m_Drivetrain.m_odometry.ResetPosition(
    frc::Rotation2d{units::degree_t {m_Drivetrain.GetYaw()}},
    {m_Drivetrain.m_frontLeft.GetPosition(), m_Drivetrain.m_frontRight.GetPosition(),
     m_Drivetrain.m_backLeft.GetPosition(),  m_Drivetrain.m_backRight.GetPosition()},
    frc::Pose2d{}
  );
  m_initialPose = m_Drivetrain.m_odometry.GetPose();
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

