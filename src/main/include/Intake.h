
#include <ctre/Phoenix.h>
#include <rev/cansparkmax.h>
#include <Constants.h>

class Intake
{
    void updateIntake (/*Phoenix!??!?!?! Harry Potter reference?????????*/);
rev::CANSparkMax m_IntakeMotor { ConstantCrap::kIntakeMotorcanID,rev::CANSparkLowLevel::MotorType::kBrushless };
};
