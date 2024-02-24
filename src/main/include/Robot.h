#pragma once

#include <string>
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Phoenix5.h>
#include <Drivetrain.h>
#include <Shooter.h>
#include <Arm.h>
#include <Climber.h>
#include <Intake.h>
//#include <frc/PowerDistribution.h>


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
    void Drivetrain_Drive(units::meters_per_second_t  xSpeed,
                        units::radians_per_second_t rot );
    void Drivetrain_Stop();
    bool DriveForDistance( units::meters_per_second_t speed, units::meter_t distance, units::time::second_t maxTime );
    bool RunDriveAuto();

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


    Drivetrain m_swerve;
    Arm        m_Arm;
    Intake     m_Intake;
    Climber    m_Climber;
    Shooter    m_Shooter;

    //int m_Count=0;
    // std::string m_smart="idk";
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
    // to 1.
    frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{5 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{5 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_rotLimiter{6 / 1_s};
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

};



