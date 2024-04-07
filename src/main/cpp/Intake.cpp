#include <Debug.h>
#include <Intake.h>
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake()
{
    m_IntakeMotor.SetSmartCurrentLimit(20, 35);
}

 
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

        case (Intake_IntakingWithSensor):
        {
            // Might want to include an override for the ring detector 
            // in case it stops working... Because we already have that.
            if ( IsRingDetected() )
            {
                IntakeSpeed = 0.0;
            }
            else
            {
                IntakeSpeed = kIntakeSpeed;
            }

            break;
        }
        case (Intake_Intaking):
        {
            IntakeSpeed = kIntakeSpeed;
            break;
        }
        case (Intake_Outtaking):
        { 
            IntakeSpeed = kOuttakeSpeed;
            break;
        }
        case (Intake_Stopped):
        {
            IntakeSpeed = 0.0;
            break;
        }
    }
    
   #if DBG_DISABLE_INTAKE_MOTORS
    m_IntakeMotor.Set( 0 );  
   #else
    m_IntakeMotor.Set(IntakeSpeed);
   #endif
}

void Intake::UpdateSmartDashboardData()
{
    frc::SmartDashboard::PutNumber( "Intake_Detc", m_RingDetector.GetValue() );
    frc::SmartDashboard::PutBoolean("NoteDetected", IsRingDetected() );
    #if 0
    frc::SmartDashboard::PutNumber( "Intake_State", m_intake_movement );
    frc::SmartDashboard::PutNumber( "Intake_Detc", m_RingDetector.GetValue() );
    #endif
}



bool Intake::IsRingDetected() 
{   
    //Good range seems like it should be about 1600?
    return (m_RingDetector.GetValue() > 1600);
}