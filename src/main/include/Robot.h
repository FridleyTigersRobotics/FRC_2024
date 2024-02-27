#pragma once

#include <string>
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <Phoenix5.h>
#include <Drivetrain.h>
#include <Shooter.h>
#include <Arm.h>
#include <Climber.h>
#include <Intake.h>
//#include <frc/PowerDistribution.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include "units/angular_acceleration.h"

int GetAnalogChannelFromPin( int io_pin_number );

class Robot : public frc::TimedRobot {
 public:
    void RobotInit() override;
    void RobotPeriodic() override;

    void DisabledInit() override;

    void TestInit() override;
    void TestPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;




    // Autonomous
    void Drivetrain_Stop();
    void DriveForDistance( 
      units::meter_t                xDistance, 
      units::meter_t                yDistance, 
      units::radian_t               rotRadians, 
      units::meters_per_second_t  xSpeedMult, 
      units::meters_per_second_t  ySpeedMult, 
      units::radians_per_second_t rotSpeed,
      units::time::second_t       maxTime
   );
   void MoveArmForPickup();
   void MoveArmForShooting();
   void AimAndPrepShoot( units::second_t maxTime );
   void Shoot( units::second_t maxTime );
   void Wait( units::second_t maxTime );

    void RunAutoSequence();
    void AutonomousStateInit();
    void AutonomousStateUpdate();


    // Teleop
   void DriveWithJoystick(
      bool fieldRelative,
      double xSpeedInput,
      double ySpeedInput,
      double rotInput
   );


 private:
    frc::XboxController m_driveController{0};
    frc::XboxController m_coController   {1};

    // Switches co-controller to end game mode
    bool m_controlModeEndGame = false;


    Drivetrain m_Drivetrain;
    Arm        m_Arm;
    Intake     m_Intake;
    Climber    m_Climber;
    Shooter    m_Shooter;





    //int m_Count=0;
    // std::string m_smart="idk";
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
    // to 1.
    frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{2 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{2 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_rotLimiter{2 / 1_s};


    // Auto


    frc::SendableChooser<std::string> m_autoChooser;
    std::string  m_autoSelected { kAutoNameDefault };
    frc::Timer   m_autoTimer;
    int          m_autoSequence { 0 };
    bool         m_initState    { true };
    //frc::Pose2d  m_initialPose;



    double m_xyDirP      = 1.000;  
    double m_rotP        = 1.000;  
    units::meters_per_second_t          m_xyMaxVel    { std::numbers::pi * 1_mps };
    units::meters_per_second_squared_t  m_xyMaxAccel  { std::numbers::pi * 2_mps / 1_s };
    units::radians_per_second_t         m_rotMaxVel   { std::numbers::pi * 1_rad_per_s };
    units::radians_per_second_squared_t m_rotMaxAccel { std::numbers::pi * 2_rad_per_s / 1_s };

    units::meter_t             kXyPosTolerance{ 0.05_m };
    units::meters_per_second_t kXyVelTolerance{ 0.5_mps };

    units::radian_t             kRotPosTolerance{ 0.05_rad };
    units::radians_per_second_t kRotVelTolerance{ 0.5_rad_per_s };

    units::meter_t m_startXArmPosition{ 0.0 };

    frc::PIDController m_xDirPid2{
      1.0,
      0.0,
      0.0
    };

    frc::ProfiledPIDController<units::meter> m_xDirPid{
      m_xyDirP,
      0.0,
      0.0,
      {units::meters_per_second_t{m_xyMaxVel}, units::meters_per_second_squared_t{m_xyMaxAccel}}
    };

    frc::ProfiledPIDController<units::meter> m_yDirPid{
      m_xyDirP,
      0.0,
      0.0,
      {units::meters_per_second_t{m_xyMaxVel}, units::meters_per_second_squared_t{m_xyMaxAccel}}
    };

    frc::ProfiledPIDController<units::radians> m_rotPid{
      m_rotP,
      0.0,
      0.0,
      {units::radians_per_second_t{m_rotMaxVel}, units::radians_per_second_squared_t{m_rotMaxAccel}}
    };


  std::vector<std::function<void(void)>> IntakeAutoTest = {
    [this] (void) -> void { MoveArmForPickup(); },
    [this] (void) -> void { Wait( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( -1.0_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForShooting(); },
    [this] (void) -> void { Wait( 1.0_s ); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
    [this] (void) -> void { Drivetrain_Stop(); m_Shooter.changeShooterState( false ); },
  };






    unsigned int m_autoState    { 0 }; 
    bool m_autoStateDone    { false }; 

    double m_prevAngle             { 0 };
    double m_currentAngle          { 0 };
    double m_angleDelta            { 0 };
    double m_currentAvgAngle       { 0 };
    double m_avgAngleDelta         { 0 };
    double m_prevAvgAngle          { 0 };
    int m_atRotateSetpointCount    { 0 };


    const std::string kAutoNameDefault  { "DO NOTHING" };
    const std::string kAutoDrive        { "Drive" };
    const std::string kShootCenter        { "ShootCenter" };
    const std::string kShootCenterPickupCenter { "ShootCenterPickupCenter" };
    const std::string kShootLeftPickupLeft { "ShootLeftPickupLeft" };
    const std::string kShootRightPickupRight { "ShootRightPickupRight" };


  std::vector<std::function<void(void)>> defaultAutoSequence = {
    [this] (void) -> void { Drivetrain_Stop(); },
  };

  // TESTED
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
    [this] (void) -> void { Wait( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( -1.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForShooting(); },
    [this] (void) -> void { DriveForDistance( 1.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s); },
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
    [this] (void) -> void { DriveForDistance( 0.0_m, -0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m,   0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },
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



  std::vector<std::function<void(void)>> *autoSequence{ &defaultAutoSequence };

};



