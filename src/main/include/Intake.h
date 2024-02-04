
#include <Phoenix5.h>
#include <rev/cansparkmax.h>
#include <Constants.h>

typedef enum intake_movement
{
    Intake_Intaking,
    Intake_Outtaking,
    Intake_Stopped,

} intake_movement_t;


class Intake{

    void ChangeIntakeState(intake_movement_t);
    intake_movement_t m_intake_movement;
    void updateIntake ();
rev::CANSparkMax m_IntakeMotor { ConstantCrap::kIntakeMotorcanID,rev::CANSparkLowLevel::MotorType::kBrushless };

};
