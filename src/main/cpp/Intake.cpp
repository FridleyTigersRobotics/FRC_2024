#include <Intake.h>

void Intake::initIntake()
{

}

 /*Hey there ;)*/
void Intake::ChangeIntakeState(intake_movement_t IntakeState)
{
    m_intake_movement=IntakeState;

}

void Intake::updateIntake()
{
    double IntakeSpeed = 0;
    switch (m_intake_movement)
    {
        case (Intake_Intaking):
        {
            IntakeSpeed = 1;
            break;
        }
        case (Intake_Outtaking):
        {
            IntakeSpeed = -1;
            break;
        }
        case (Intake_Stopped):
        {
            IntakeSpeed = 0;
            break;
        }
    }

    if (IsRingNotDetected())
    {
        m_IntakeMotor.Set(IntakeSpeed);
    }
    else
     {
        if ( IntakeSpeed > 0.0 )
        {
            m_IntakeMotor.Set(IntakeSpeed);
        }
    }
    
     
 
}