#include <rev/cansparkmax.h>
#include <Constants.h>
class Shooter
{
void updateShooter (/*among us*/);

rev::CANSparkMax m_ShooterMotor { ConstantCrap::kShooterMotorID,rev::CANSparkLowLevel::MotorType::kBrushless };

};