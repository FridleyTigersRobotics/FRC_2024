#include <Debug.h>
#include <Climber.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Climber::initClimber()
{
    m_leftClimberMotor.SetInverted( false );
    m_rightClimberMotor.SetInverted( true );
}

void Climber::updateClimber()
{ /*Is that freddy five bear? Hor hor hor hor hor*/ 

    double ClimberMotorSpeed = 0;
    switch (m_ClimberState)
    {
        case (ClimberUp):
        {
            ClimberMotorSpeed = 1;
            break;
        }
        case (ClimberDown):
        { // TODO : Might want lower speed for down, need to test.
            ClimberMotorSpeed = -1;
            break;
        }
        case (ClimberStop):
        {
            ClimberMotorSpeed = 0;
            break;
        }
    }

    frc::SmartDashboard::PutNumber( "Climber_Output", ClimberMotorSpeed );

   #if DBG_DISABLE_CLIMB_MOTORS
    m_leftClimberMotor.Set( 0 );
    m_rightClimberMotor.Set( 0 );  
   #else
    m_leftClimberMotor.Set(ClimberMotorSpeed);
    m_rightClimberMotor.Set(ClimberMotorSpeed);  
   #endif
}

void Climber::ChangeClimberState( ClimberState_t ClimberState )
{
    m_ClimberState = ClimberState;
}