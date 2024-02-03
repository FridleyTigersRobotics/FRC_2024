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