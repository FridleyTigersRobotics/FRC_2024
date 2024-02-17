#pragma once

#include <Phoenix5.h>
#include <rev/cansparkmax.h>
#include <Constants.h>
#include <frc/AnalogInput.h>

typedef enum intake_movement
{
    Intake_Intaking,
    Intake_Outtaking,
    Intake_Stopped,

} intake_movement_t;


class Intake{
 public:
    void initIntake();
    void ChangeIntakeState(intake_movement_t);
    void updateIntake ();
    bool IsRingDetected();

  private:
    rev::CANSparkMax m_IntakeMotor { ConstantCrap::kIntakeMotorcanID,rev::CANSparkLowLevel::MotorType::kBrushless };
    frc::AnalogInput m_RingDetector{0};
    intake_movement_t m_intake_movement;

};
