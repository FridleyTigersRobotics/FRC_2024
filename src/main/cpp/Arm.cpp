#include <Arm.h>
#include <rev/cansparkmax.h>
//#include <arms> *flexes, cutely*
void Arm::SetArmPosition (arm_position_t DesiredPosition)
{
m_ArmPosition = DesiredPosition;



}

void Arm::updateArm()
{
 switch (m_ArmPosition)
    {
        case (GROUND_PICKUP):
        {

            break;
        }
        case (SOURCE):
        {

            break;
        }
        case (SPEAKER):
        {

            break;
        }
        case (AMP):
        {

            break;
        }
        case (TRAP):
        {

            break;
        }
    }
}
//Change to work for wrist???
void Arm::updateWrist()
{
 switch (m_WristPosition)
    {
        case (WRIST_GROUND_PICKUP):
        {

            break;
        }
        case (WRIST_SOURCE):
        {

            break;
        }
        case (WRIST_SHOOT):
        {

            break;
        }
        case (WRIST_AMP):
        {

            break;
        }
        case (WRIST_TRAP):
        {

            break;
        }
    }


}