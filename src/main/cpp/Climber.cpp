#include <Debug.h>
#include <Climber.h>
#include <frc/smartdashboard/SmartDashboard.h>




void Climber::initClimber()
{
    m_leftClimberMotor.SetInverted( false );
    m_rightClimberMotor.SetInverted( false );

    m_motorEncoderL.SetMinRate(1);
    m_motorEncoderL.SetDistancePerPulse(0.1);
    m_motorEncoderL.SetReverseDirection(false);
    m_motorEncoderL.SetSamplesToAverage(2);
    m_motorEncoderL.SetReverseDirection( true );
    m_motorEncoderL.Reset();

    m_motorEncoderR.SetMinRate(1);
    m_motorEncoderR.SetDistancePerPulse(0.1);
    m_motorEncoderR.SetReverseDirection(true);
    m_motorEncoderR.SetSamplesToAverage(2);
    m_motorEncoderR.SetReverseDirection( true );
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
            if ( m_ClimberPosition > 0.0 )
            {
                m_ClimberPosition = 0.0;
            }
           #else
            ClimberMotorSpeed = -1.0;
           #endif
            break;
        }
        case (ClimberDown):
        {
           #if CLIMBER_ENCODER_SYNC_ENABLED
            m_ClimberPosition -= kCLimberSpeed;
            /*if ( m_ClimberPosition < -3.5e5 )
            {
                m_ClimberPosition = -2.7e5;
            }*/
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
   
   #if CLIMBER_GYRO_ENABLED
    double offset = kAngleToEncoderCounts * sin( m_roll );
   #else
    double offset = 0.0;
   #endif
    bool   applyToRight = offset > 0;

    double positionR = m_ClimberPosition + (  applyToRight ? ( offset ) : ( 0.0 ) );
    double positionL = m_ClimberPosition + ( !applyToRight ? ( offset ) : ( 0.0 ) );

    m_ClimberLeftPidController.SetSetpoint( positionL );
    m_ClimberRightPidController.SetSetpoint( positionR );

    double ClimberMotorSpeedL = -m_ClimberLeftPidController.Calculate(  m_motorEncoderL.Get() );
    double ClimberMotorSpeedR = -m_ClimberRightPidController.Calculate( m_motorEncoderR.Get() );
   #else
    double ClimberMotorSpeedL = ClimberMotorSpeed;
    double ClimberMotorSpeedR = ClimberMotorSpeed;
   #endif

    if ( ClimberMotorSpeedL > 0.0 && m_leftLimitSwitch.Get() )
    {
        ClimberMotorSpeedL = 0;
    }

    if ( ClimberMotorSpeedR > 0.0 && m_rightLimitSwitch.Get() )
    {
        ClimberMotorSpeedR = 0;
    }

    #if !CLIMBER_MANUAL_CONTROL
    m_leftClimberMotor.Set(  std::clamp( ClimberMotorSpeedL, -m_ClimberLMaxOutputValue, m_ClimberLMaxOutputValue ) );
    m_rightClimberMotor.Set( std::clamp( ClimberMotorSpeedR, -m_ClimberRMaxOutputValue, m_ClimberRMaxOutputValue ) );
    #endif

   #endif
}

void Climber::ChangeClimberState( ClimberState_t ClimberState )
{
    m_ClimberState = ClimberState;
}


void Climber::UpdateSmartDashboardData( )
{
#if 0
    frc::SmartDashboard::PutNumber( "Climber_State", m_ClimberState );
    frc::SmartDashboard::PutNumber( "Climber_OutputL", m_leftClimberMotor.Get() );
    frc::SmartDashboard::PutNumber( "Climber_OutputR", m_rightClimberMotor.Get() );
    frc::SmartDashboard::PutNumber( "Climber_LimitL", m_leftLimitSwitch.Get() );
    frc::SmartDashboard::PutNumber( "Climber_LimitR", m_rightLimitSwitch.Get() );

    frc::SmartDashboard::PutNumber( "Climber_EncoderL", m_motorEncoderL.Get() );
    frc::SmartDashboard::PutNumber( "Climber_EncoderR", m_motorEncoderR.Get() );
    frc::SmartDashboard::PutNumber( "Climber_prevRoll", m_prevRoll );
    frc::SmartDashboard::PutNumber( "Climber_roll",     m_roll );
#endif




   #if 0 && CLIMBER_ENCODER_SYNC_ENABLED
   frc::SmartDashboard::PutNumber( "Climber_LeftError", m_ClimberLeftPidController.GetPositionError() );
   frc::SmartDashboard::PutNumber( "Climber_RightError", m_ClimberRightPidController.GetPositionError() );
    frc::SmartDashboard::PutNumber( "Climber_ClimberPosition", m_ClimberPosition );
   #endif
}
