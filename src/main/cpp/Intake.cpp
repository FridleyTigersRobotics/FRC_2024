#include <Intake.h>

void Intake::initIntake()
{

}

 /*Hey there ;)*/
void Intake::ChangeIntakeState(intake_movement_t IntakeState)
{
    m_intake_movement = IntakeState;
}

void Intake::updateIntake()
{
    double IntakeSpeed = 0.0;

    switch (m_intake_movement)
    {
        case (Intake_Intaking):
        {
            // Might want to include an override for the ring detector 
            // in case it stops working...
            if ( IsRingDetected() )
            {
                IntakeSpeed = 0.0;
            }
            else
            { // TODO : Determine intaking speed & direction
                IntakeSpeed = 1.0;
            }

            break;
        }
        case (Intake_Outtaking):
        { // TODO : Determine out speed & direction
            IntakeSpeed = -1.0;
            break;
        }
        case (Intake_Stopped):
        {
            IntakeSpeed = 0.0;
            break;
        }
    }

    m_IntakeMotor.Set(IntakeSpeed);
}


bool Intake::IsRingDetected() 
{   
    // TODO : determine the correct value to detect note
    return m_RingDetector.GetValue() < 50;
}