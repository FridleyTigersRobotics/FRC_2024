#include <Shooter.h>
#include <Intake.h>

void Shooter::updateShooter( bool spinUpShooter )
{
    if ( spinUpShooter /*&& IsRingNotDetected()*/)
    {
        m_ShooterMotor.Set(1);
    }
    else
    {
        m_ShooterMotor.Set(0);
    }
}