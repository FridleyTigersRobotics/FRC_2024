#include <Debug.h>
#include <Climber.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Climber::initClimber()
{
    m_leftClimberMotor.SetInverted( false );
    m_rightClimberMotor.SetInverted( true );

    //m_motorEncoderL.SetMaxPeriod(units::time::second_t{1});
    m_motorEncoderL.SetMinRate(1);
    m_motorEncoderL.SetDistancePerPulse(0.1);
    m_motorEncoderL.SetReverseDirection(false);
    m_motorEncoderL.SetSamplesToAverage(2);
    m_motorEncoderL.Reset();

    //m_motorEncoderR.SetMaxPeriod(units::time::second_t{1});
    m_motorEncoderR.SetMinRate(1);
    m_motorEncoderR.SetDistancePerPulse(0.1);
    m_motorEncoderR.SetReverseDirection(true);
    m_motorEncoderR.SetSamplesToAverage(2);
    m_motorEncoderR.Reset();

}

void Climber::manualControl( double speedL, double speedR )
{
    m_leftClimberMotor.Set(  0.4 * speedL );
    m_rightClimberMotor.Set( 0.4 * speedR );  
}

void Climber::updateClimber()
{ /*Is that freddy five bear? Hor hor hor hor hor*/ 
    double ClimberMotorSpeed = 0;
    switch (m_ClimberState)
    {
        case (ClimberUp):
        {
            ClimberMotorSpeed = -1.0;
            break;
        }
        case (ClimberDown):
        { // TODO : Might want lower speed for down, need to test.
            ClimberMotorSpeed = 1.0;
            break;
        }
        case (ClimberStop):
        default:
        {
            ClimberMotorSpeed = 0;
            break;
        }
    }

   #if DBG_DISABLE_CLIMB_MOTORS
    m_leftClimberMotor.Set( 0 );
    m_rightClimberMotor.Set( 0 );  
   #else
    double ClimberMotorSpeedL = ClimberMotorSpeed;
    double ClimberMotorSpeedR = ClimberMotorSpeed;

    if( m_ClimberState == ClimberDown ) 
    {
        if ( m_leftLimitSwitch.Get() )
        {
            ClimberMotorSpeedL = 0;
        }

        if ( m_rightLimitSwitch.Get() )
        {
            ClimberMotorSpeedR = 0;
        }
    }

    m_leftClimberMotor.Set(ClimberMotorSpeedL);
    m_rightClimberMotor.Set(ClimberMotorSpeedR);  

   #endif
}

void Climber::ChangeClimberState( ClimberState_t ClimberState )
{
    m_ClimberState = ClimberState;
}


void Climber::UpdateSmartDashboardData( )
{
    frc::SmartDashboard::PutNumber( "Climber_State", m_ClimberState );
    frc::SmartDashboard::PutNumber( "Climber_OutputL", m_leftClimberMotor.Get() );
    frc::SmartDashboard::PutNumber( "Climber_OutputR", m_rightClimberMotor.Get() );
    frc::SmartDashboard::PutNumber( "Climber_LimitL", m_leftLimitSwitch.Get() );
    frc::SmartDashboard::PutNumber( "Climber_LimitR", m_rightLimitSwitch.Get() );

    frc::SmartDashboard::PutNumber( "Climber_EncoderL", m_motorEncoderL.Get() );
    frc::SmartDashboard::PutNumber( "Climber_EncoderR", m_motorEncoderR.Get() );
}
