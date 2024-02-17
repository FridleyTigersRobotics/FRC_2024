
#include <Climber.h>

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
        {
            ClimberMotorSpeed = -1;
            break;
        }
        case (ClimberStop):
        {
            ClimberMotorSpeed = 0;
            break;
        }
    }
    m_leftClimberMotor.Set(ClimberMotorSpeed);
    m_rightClimberMotor.Set(ClimberMotorSpeed);  
}

void Climber::ChangeClimberState( ClimberState_t ClimberState )
{
    m_ClimberState = ClimberState;
}