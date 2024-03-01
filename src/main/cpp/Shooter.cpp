#include <Debug.h>
#include <Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/printf.h>

Shooter::Shooter()
{
    m_shooterMotor.RestoreFactoryDefaults();
    m_shooterMotor.SetInverted( true );
    m_shooterPid.SetP(kP);
    m_shooterPid.SetI(kI);
    m_shooterPid.SetD(kD);
    m_shooterPid.SetIZone(kIz);
    m_shooterPid.SetFF(kFF);
    m_shooterPid.SetOutputRange(kMinOutput, kMaxOutput);
    m_shooterPid.SetSmartMotionMaxVelocity(kMaxVel);
    m_shooterPid.SetSmartMotionMinOutputVelocity(kMinVel);
    m_shooterPid.SetSmartMotionMaxAccel(kMaxAcc);
    m_shooterPid.SetSmartMotionAllowedClosedLoopError(kAllErr);
}

void Shooter::initShooter()
{

#if 0
    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    frc::SmartDashboard::PutNumber("Max Velocity", kMaxVel);
    frc::SmartDashboard::PutNumber("Min Velocity", kMinVel);
    frc::SmartDashboard::PutNumber("Max Acceleration", kMaxAcc);
    frc::SmartDashboard::PutNumber("Allowed Closed Loop Error", kAllErr);
    frc::SmartDashboard::PutNumber("Set Position", 0);
    frc::SmartDashboard::PutNumber("Set Velocity", 0);
    #endif

}

void Shooter::changeShooterState( bool spinUpShooter )
{
    m_spinUpShooter = spinUpShooter;
}


bool Shooter::shooterReadyToShoot()
{
    return m_shooterEncoder.GetVelocity() > ( m_maxShooterSpeed - 500 );
}

void Shooter::updateShooter( )
{
    double const shooterVelocity      = m_shooterEncoder.GetVelocity();
    double const shooterSpeedAbsError = m_maxShooterSpeed - shooterVelocity;
    // TODO : determine this delta for switching to PID control.
    double const speedDeltaForPidControl = 200;


    frc::SmartDashboard::PutBoolean( "ShooterUpToSpeed", ( shooterVelocity / m_maxShooterSpeed ) > 0.95 );

#if 0
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double maxV = frc::SmartDashboard::GetNumber("Max Velocity", 0);
    double minV = frc::SmartDashboard::GetNumber("Min Velocity", 0);
    double maxA = frc::SmartDashboard::GetNumber("Max Acceleration", 0);
    double allE = frc::SmartDashboard::GetNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP))   { m_shooterPid.SetP(p); kP = p; }
    if((i != kI))   { m_shooterPid.SetI(i); kI = i; }
    if((d != kD))   { m_shooterPid.SetD(d); kD = d; }
    if((iz != kIz)) { m_shooterPid.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_shooterPid.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { m_shooterPid.SetOutputRange(min, max); kMinOutput = min; kMaxOutput = max; }
    if((maxV != kMaxVel)) { m_shooterPid.SetSmartMotionMaxVelocity(maxV); kMaxVel = maxV; }
    if((minV != kMinVel)) { m_shooterPid.SetSmartMotionMinOutputVelocity(minV); kMinVel = minV; }
    if((maxA != kMaxAcc)) { m_shooterPid.SetSmartMotionMaxAccel(maxA); kMaxAcc = maxA; }
    if((allE != kAllErr)) {  m_shooterPid.SetSmartMotionAllowedClosedLoopError(allE); allE = kAllErr; }
#endif


    // TODO : Determine this speed error that we can shoot at.
    if ( shooterSpeedAbsError < 100 )
    {
        m_shooterSpeedReadyToShoot = true;
    }
    else
    {
        m_shooterSpeedReadyToShoot = false;
    }

   #if DBG_DISABLE_SHOOT_MOTORS
    m_shooterMotor.Set( 0 );  
   #else
    if ( m_spinUpShooter )
    {   
        if ( shooterVelocity < ( m_maxShooterSpeed - speedDeltaForPidControl ) )
        {   
            double const motorOutput = m_AccelerationLimiter.Calculate( 1.0 ) * m_maxAccelOutput;
            m_shooterMotor.Set( motorOutput );
        }
        else
        {
            m_shooterPid.SetReference( m_maxShooterSpeed, rev::CANSparkBase::ControlType::kVelocity );
        }
    }
    else
    {
        m_AccelerationLimiter.Reset( 0 );
        m_shooterMotor.Set( 0.0 );
    }
   #endif

}


void Shooter::UpdateSmartDashboardData()
{
    #if 0
    frc::SmartDashboard::PutNumber( "Shooter_Speed",        m_shooterEncoder.GetVelocity() );
    frc::SmartDashboard::PutNumber( "Shooter_Output",       m_shooterMotor.GetAppliedOutput() );
    frc::SmartDashboard::PutNumber( "Shooter_Current",      m_shooterMotor.GetOutputCurrent() );
    frc::SmartDashboard::PutNumber( "Shooter_ReadyToShoot", ReadyToShoot() );
    #endif
}


bool Shooter::ReadyToShoot()
{
    return m_shooterSpeedReadyToShoot;
}

