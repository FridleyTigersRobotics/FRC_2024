#include <Arm.h>
#include <rev/cansparkmax.h>
#include <frc/smartdashboard/SmartDashboard.h>
//#include <arms> *flexes, cutely*
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
    const auto ArmControllOutput = m_ArmPIDController.Calculate(
        units::radian_t{m_ArmEncoder.GetDistance()}, units::radian_t{ArmAngle});
    m_ArmMotorLeft.Set(ArmControllOutput);
    m_ArmMotorRight.Set(ArmControllOutput);
    frc::SmartDashboard::PutNumber("Arm_ArmControllOutput", ArmControllOutput);
    frc::SmartDashboard::PutNumber("ArmAngle", ArmAngle);
    frc::SmartDashboard::PutNumber("ArmPosition", m_ArmPosition);


    const auto WristControllOutput = m_WristPIDController.Calculate(
        units::radian_t{m_WristEncoder.GetDistance()}, units::radian_t{WristAngle});
    m_WristMotor.Set(WristControllOutput);

   /* const auto WristAngle = m_WristPIDController.Calculate(
        units::radian_t{m_WristEncoder.GetDistance()}, units::radian_t{WristAngle});*/
frc::SmartDashboard::PutNumber("Wrist_WristControllOutput", WristControllOutput);
frc::SmartDashboard::PutNumber("WristAngle", WristAngle);
frc::SmartDashboard::PutNumber("WristPosition", m_WristPosition);
}

//Change to work for wrist???

//PID? PIN? Bowling? Wii Sports Bowling???
