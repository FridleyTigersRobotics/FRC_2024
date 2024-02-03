#include <Arm.h>
#include <rev/cansparkmax.h>
//#include <arms> *flexes, cutely*
void Arm::SetArmPosition (arm_position_t DesiredPosition)
{
m_ArmPosition = DesiredPosition;



}

void Arm::updateArm()
{
    double ArmAngle = 0;

 switch (m_ArmPosition)
    {
        case (GROUND_PICKUP):
        {
            ArmAngle = 0;
            break;
        }
        case (SOURCE):
        {
            ArmAngle = 0.4;
            break;
        }
        case (SPEAKER):
        {
            ArmAngle = 0;
            break;
        }
        case (AMP):
        {
            ArmAngle = 0.5;
            break;
        }
        case (TRAP):
        {
            ArmAngle = 1.0;
            break;
        }
    }
//PIDdly thing
const auto ArmControllOutput = m_ArmPIDController.Calculate(
      units::radian_t{m_ArmEncoder.GetDistance()}, units::radian_t{ArmAngle});
m_ArmMotorLeft.Set(ArmControllOutput);
m_ArmMotorRight.Set(ArmControllOutput);
}

//Change to work for wrist???
void Arm::updateWrist()
{
    double WristAngle = 0;
 switch (m_WristPosition)
    {
        case (WRIST_GROUND_PICKUP):
        {

            WristAngle = 1;
            break;

        }
        case (WRIST_SOURCE):
        {

            WristAngle = 0.3;
            break;

        }
        case (WRIST_SHOOT):
        {

            WristAngle = 0;
            break;

        }
        case (WRIST_AMP):
        {

            WristAngle = 0.7;
            break;

        }
        case (WRIST_TRAP):
        {

            WristAngle = 0.5;
            break;

        }
    }
//PIDdly thing FOR WRIST THIS TIME
const auto WristControllOutput = m_WristPIDController.Calculate(
      units::radian_t{m_WristEncoder.GetDistance()}, units::radian_t{WristAngle});
m_WristMotor.Set(WristControllOutput);
}