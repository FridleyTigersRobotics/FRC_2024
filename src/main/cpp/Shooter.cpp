#include <Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Shooter::initShooter()
{
  m_shooterMotor.RestoreFactoryDefaults();
  //m_shooterMotor.SetInverted( true );

}

void Shooter::updateShooter( bool spinUpShooter )
{
    double const shooterVelocity      = m_shooterEncoder.GetVelocity();
    double const shooterSpeedAbsError = m_maxShooterSpeed - shooterVelocity;

    // TODO : Determine this speed error that we can shoot at.
    if ( shooterSpeedAbsError < 100 )
    {
        m_shooterSpeedReadyToShoot = true;
    }
    else
    {
        m_shooterSpeedReadyToShoot = false;
    }

    if ( spinUpShooter )
    {   // TODO : determine this delta for switching to PID control.
        double const speedDeltaForPidControl = 500;

        if ( shooterVelocity < ( m_maxShooterSpeed - speedDeltaForPidControl ) )
        {   
            double const motorOutput = m_AccelerationLimiter.Calculate( 1.0 ) * m_maxAccelOutput;
            m_shooterMotor.Set( motorOutput );
        }
        else
        {
            m_shooterPid.SetReference( m_maxShooterSpeed, rev::CANSparkBase::ControlType::kVelocity, 0 );
        }
    }
    else
    {
        m_shooterMotor.Set( 0.0 );
    }

    frc::SmartDashboard::PutNumber( "Shooter Speed",  shooterVelocity );
    frc::SmartDashboard::PutNumber( "Shooter Output", m_shooterMotor.GetAppliedOutput() );
}


bool Shooter::ReadyToShoot()
{
    return m_shooterSpeedReadyToShoot;
}

