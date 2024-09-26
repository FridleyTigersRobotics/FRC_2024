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

void Robot::TestInit()
{
    m_Arm.disableArm();
}

void Robot::TestPeriodic()
{

    m_Arm.SetArmPosition(m_Arm.HOLD_START_POSITION);
    m_Intake.ChangeIntakeState(m_Intake.Intake_Stopped);
    m_Drivetrain.SetSpeeds(0.0_mps, 0.0_mps, 0.0_rad_per_s);

    double speedL = 0;
    double speedR = 0;

    //if (m_coController.GetLeftBumper())
    if (m_buttons.GetRawButton(1))
    {
        speedL = 1.0;
    }
    //if (m_coController.GetRightBumper())
    if (m_buttons.GetRawButton(2))
    {
        speedR = 1.0;
    }

    //if (m_coController.GetLeftTriggerAxis() > 0.2)
    if (m_buttons.GetRawButton(3))
    {
        speedL = -1.0;
    }
    //if (m_coController.GetRightTriggerAxis() > 0.2)
    if (m_buttons.GetRawButton(4))
    {
        speedR = -1.0;
    }

    m_Climber.manualControl(speedL, speedR);
}
