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
#include <Auto.h>


std::vector<std::function<void(void)>> Auto_Drive = {
    [this] (void) -> void { DriveForDistance( -2.0_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { Drivetrain_Stop(); },
  };

  // TESTED
  std::vector<std::function<void(void)>> Auto_ShootCenter = {
    //[this] (void) -> void { DriveForDistance( 0.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
    [this] (void) -> void { Drivetrain_Stop(); m_Shooter.changeShooterState( false ); },
  };

//TESTED :D :D :D
  std::vector<std::function<void(void)>> Auto_ShootCenterPickupCenter = {
    //[this] (void) -> void { DriveForDistance( 0.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
    [this] (void) -> void { m_Shooter.changeShooterState( false ); m_autoStateDone = true; },
    [this] (void) -> void { MoveArmForPickup(); },
    //[this] (void) -> void { Wait( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( -1.5_m, 0.0_m, 0.0_rad, 0.9_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForShooting(); },
    [this] (void) -> void { DriveForDistance( 1.5_m, 0.0_m, 0.0_rad, 0.8_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
    [this] (void) -> void { Drivetrain_Stop(); m_Shooter.changeShooterState( false ); },
  };


  std::vector<std::function<void(void)>> ShootLeftPickupLeft = {
    //[this] (void) -> void { DriveForDistance( 0.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
    [this] (void) -> void { m_Shooter.changeShooterState( false ); m_autoStateDone = true; },
    //[this] (void) -> void { Wait( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( -0.7_m, 0.0_m, 0.0_rad, 0.85_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, 1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForPickup(); },
    [this] (void) -> void { DriveForDistance( -0.2_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( -1.33_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    // [this] (void) -> void { DriveForDistance( 0.0_m, -0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },
    // [this] (void) -> void { DriveForDistance( 0.0_m,   0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForShooting(); },
    [this] (void) -> void { DriveForDistance( 1.53_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, -1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.61_m, 0.0_m, 0.0_rad, 0.85_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, -0.1_m, 0.0_rad, 0.0_mps, 0.5_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AimAndPrepShoot( 2.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
    [this] (void) -> void { Drivetrain_Stop(); m_Shooter.changeShooterState( false ); },

  };

  std::vector<std::function<void(void)>> ShootRightPickupRight = {
    //[this] (void) -> void { DriveForDistance( 0.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
    [this] (void) -> void { m_Shooter.changeShooterState( false ); m_autoStateDone = true; },
    //[this] (void) -> void { Wait( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( -0.7_m, 0.0_m, 0.0_rad, 0.85_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, -1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForPickup(); },
    [this] (void) -> void { DriveForDistance( -0.2_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( -1.33_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    //[this] (void) -> void { DriveForDistance( 0.0_m, -0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },
    //[this] (void) -> void { DriveForDistance( 0.0_m,   0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForShooting(); },
    [this] (void) -> void { DriveForDistance( 1.55_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, 1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.61_m, 0.0_m, 0.0_rad, 0.85_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, -0.1_m, 0.0_rad, 0.0_mps, 0.5_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AimAndPrepShoot( 2.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
    [this] (void) -> void { Drivetrain_Stop(); m_Shooter.changeShooterState( false ); },
  };

  std::vector<std::function<void(void)>> Auto_CenterShootRun = {
    //[this] (void) -> void { DriveForDistance( 0.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
    [this] (void) -> void { m_Shooter.changeShooterState( false ); m_autoStateDone = true; },
    [this] (void) -> void { MoveArmForPickup(); },
    //[this] (void) -> void { Wait( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( -1.5_m, 0.0_m, 0.0_rad, 0.9_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForShooting(); },
    [this] (void) -> void { DriveForDistance( 1.5_m, 0.0_m, 0.0_rad, 0.8_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
    [this] (void) -> void {m_Shooter.changeShooterState( false ); m_autoStateDone = true; },
    [this] (void) -> void {( -1.5_m, 0.0_m, 0.0_rad, 0.9_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s); },
    [this] (void) -> void { Drivetrain_Stop();},
  };*/