#include <Debug.h>
#include <Intake.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Intake::initIntake()
{
    m_intake_movement = Intake_Stopped;
}

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
                IntakeSpeed = -0.4;
            }

            break;
        }
        case (Intake_Outtaking):
        { // TODO : Determine out speed & direction
            IntakeSpeed = 1.0;
            break;
        }
        case (Intake_Stopped):
        {
            IntakeSpeed = 0.0;
            break;
        }
    }

    frc::SmartDashboard::PutNumber( "Intake_Output", IntakeSpeed );
    //frc::SmartDashboard::PutNumber( "Intake_State", m_intake_movement );
    //frc::SmartDashboard::PutNumber( "Intake_Detc", m_RingDetector.GetValue() );
    
   #if DBG_DISABLE_INTAKE_MOTORS
    m_IntakeMotor.Set( 0 );  
   #else
    m_IntakeMotor.Set(IntakeSpeed);
   #endif
}

void Intake::UpdateSmartDashboardData()
{
    frc::SmartDashboard::PutNumber( "Intake_State", m_intake_movement );
    frc::SmartDashboard::PutNumber( "Intake_Detc", m_RingDetector.GetValue() );
}



bool Intake::IsRingDetected() 
{   
    // TODO : determine the correct value to detect note
    return (m_RingDetector.GetValue() > 1000);
}