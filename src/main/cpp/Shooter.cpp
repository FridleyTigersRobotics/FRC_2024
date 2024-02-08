#include <Shooter.h>
#include <frc/XboxController.h>
#include <Intake.h>

frc::XboxController m_controller{0};
void Shooter::updateShooter()
{
if (m_controller.GetXButton() /*&& IsRingNotDetected()*/)
{
    m_ShooterMotor.Set(1);
}
else
{
    m_ShooterMotor.Set(0);
}
}