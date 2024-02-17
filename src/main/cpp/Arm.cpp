#include <Arm.h>
#include <rev/cansparkmax.h>
#include <frc/smartdashboard/SmartDashboard.h>
//#include <arms> *flexes, cutely*

void Arm::initArm()
{

}

void Arm::SetArmPosition (arm_position_t DesiredPosition)
{
    m_ArmPosition = DesiredPosition;
}

void Arm::updateArm()
{
    double ArmAngle = 0;
    double WristAngle = 0;

    switch (m_ArmPosition)
    {
        case (GROUND_PICKUP):
        {
            ArmAngle = 0;
            WristAngle = 1;
            break;
        }
        case (SOURCE):
        {
            ArmAngle = 0.4;
            WristAngle = 0.3;
            break;
        }
        case (SPEAKER):
        {
            ArmAngle = 0;
            WristAngle = 0;
            break;
        }
        case (AMP):
        {
            ArmAngle = 0.5;
            WristAngle = 0.7;
            break;
        }
        case (TRAP):
        {
            ArmAngle = 1.0;
            WristAngle = 0.5;
            break;
        }
    }
    
    //PIDdly thing
    const auto ArmControlOutput = m_ArmPIDController.Calculate(
        units::radian_t{m_ArmEncoder.GetDistance()}, units::radian_t{ArmAngle});

    const auto WristControlOutput = m_WristPIDController.Calculate(
        units::radian_t{m_WristEncoder.GetDistance()}, units::radian_t{WristAngle});

    frc::SmartDashboard::PutNumber("Arm_ControlOutput", ArmControlOutput);
    frc::SmartDashboard::PutNumber("Arm_Angle",         ArmAngle);
    frc::SmartDashboard::PutNumber("Arm_Position",      m_ArmPosition);

    frc::SmartDashboard::PutNumber("Wrist_ControlOutput", WristControlOutput);
    frc::SmartDashboard::PutNumber("Wrist_Angle",         WristAngle);
    frc::SmartDashboard::PutNumber("Wrist_Position",      m_WristPosition);

    m_ArmMotorLeft.Set(ArmControlOutput);
    m_ArmMotorRight.Set(ArmControlOutput);
    m_WristMotor.Set(WristControlOutput);
}

//Change to work for wrist???

//PID? PIN? Bowling? Wii Sports Bowling???
