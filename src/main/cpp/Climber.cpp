#include <Debug.h>
#include <Climber.h>
#include <frc/smartdashboard/SmartDashboard.h>




void Climber::initClimber()
{
    m_leftClimberMotor.SetInverted( false );
    m_rightClimberMotor.SetInverted( true );

    m_motorEncoderL.SetMinRate(1);
    m_motorEncoderL.SetDistancePerPulse(0.1);
    m_motorEncoderL.SetReverseDirection(false);
    m_motorEncoderL.SetSamplesToAverage(2);
    m_motorEncoderL.Reset();

    m_motorEncoderR.SetMinRate(1);
    m_motorEncoderR.SetDistancePerPulse(0.1);
    m_motorEncoderR.SetReverseDirection(true);
    m_motorEncoderR.SetSamplesToAverage(2);
    m_motorEncoderR.Reset();

    m_ClimberLeftPidController.Reset();
    m_ClimberRightPidController.Reset();

    m_ClimberPosition = 0;
}

void Climber::manualControl( double speedL, double speedR )
{
    m_leftClimberMotor.Set(  0.4 * speedL );
    m_rightClimberMotor.Set( 0.4 * speedR );  
}

void Climber::UpdateRoll( double roll )
{
    m_prevRoll = m_roll;
    m_roll     = roll;
}


void Climber::updateClimber()
{ /*Is that freddy five bear? Hor hor hor hor hor*/ 
   #if !CLIMBER_ENCODER_SYNC_ENABLED
    double ClimberMotorSpeed = 0;
   #endif

    switch (m_ClimberState)
    {
        case (ClimberUp):
        {
           #if CLIMBER_ENCODER_SYNC_ENABLED
            m_ClimberPosition += kCLimberSpeed;
           #else
            ClimberMotorSpeed = -1.0;
           #endif
            break;
        }
        case (ClimberDown):
        {
           #if CLIMBER_ENCODER_SYNC_ENABLED
            m_ClimberPosition -= kCLimberSpeed;
           #else
            ClimberMotorSpeed = 1.0;
           #endif
            break;
        }
        case (ClimberStop):
        default:
        {
           #if !CLIMBER_ENCODER_SYNC_ENABLED
            ClimberMotorSpeed = 0;
           #endif
            break;
        }
    }

   #if DBG_DISABLE_CLIMB_MOTORS
    m_leftClimberMotor.Set( 0 );
    m_rightClimberMotor.Set( 0 );  
   #else




   #if CLIMBER_ENCODER_SYNC_ENABLED
    double ClimberMotorSpeedL = m_ClimberLeftPidController.Calculate( m_ClimberPosition );
    double ClimberMotorSpeedR = m_ClimberRightPidController.Calculate( m_ClimberPosition );
   #else
    double ClimberMotorSpeedL = ClimberMotorSpeed;
    double ClimberMotorSpeedR = ClimberMotorSpeed;
   #endif


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

    m_leftClimberMotor.Set(  std::clamp( ClimberMotorSpeedL, -m_ClimberLMaxOutputValue, m_ClimberLMaxOutputValue ) );
    m_rightClimberMotor.Set( std::clamp( ClimberMotorSpeedR, -m_ClimberRMaxOutputValue, m_ClimberRMaxOutputValue ) );

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
    frc::SmartDashboard::PutNumber( "Climber_prevRoll", m_prevRoll );
    frc::SmartDashboard::PutNumber( "Climber_roll",     m_roll );

   #if CLIMBER_ENCODER_SYNC_ENABLED
    frc::SmartDashboard::PutNumber( "Climber_ClimberPosition", m_ClimberPosition );
   #endif
}
