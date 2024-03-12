/*#include <Robot.h>
#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <fmt/printf.h>
#include <frc/filter/SlewRateLimiter.h>
#include <Shooter.h>
#include <Drivetrain.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>
#include <Arm.h>
#include <networktables/NetworkTable.h>








void Robot::DriveForDistance( 
  units::meter_t              xDistance, 
  units::meter_t              yDistance, 
  units::radian_t             rotRadians, 
  units::meters_per_second_t  xSpeedMult, 
  units::meters_per_second_t  ySpeedMult, 
  units::radians_per_second_t rotSpeedMult,
  units::time::second_t       maxTime
)


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

void Robot::Drivetrain_Stop() {
   m_Drivetrain.SetSpeeds( 0.0_mps, 0.0_mps, 0.0_rad_per_s );
}*/