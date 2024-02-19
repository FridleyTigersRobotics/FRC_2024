#include <Debug.h>
#include <Arm.h>
#include <rev/cansparkmax.h>
#include <frc/smartdashboard/SmartDashboard.h>
//#include <arms> *flexes, cutely*


Arm::Arm()
{
    m_ArmMotorRight.SetInverted( false );
    m_ArmMotorLeft.SetInverted( true );
    m_WristMotor.SetInverted( true );
    m_ArmMotorLeftEncoder.SetPositionConversionFactor(  1.0 / ( 20.0 * ( 74.0 / 14.0 ) ) );
    m_ArmMotorRightEncoder.SetPositionConversionFactor( 1.0 / ( 20.0 * ( 74.0 / 14.0 ) ) );

    m_ArmMotorLeftEncoder.SetVelocityConversionFactor(  1.0 / ( 60 * 20.0 * ( 74.0 / 14.0 ) ) );
    m_ArmMotorRightEncoder.SetVelocityConversionFactor( 1.0 / ( 60 * 20.0 * ( 74.0 / 14.0 ) ) );



    m_pidControllerLeft.SetP(kP);
    m_pidControllerLeft.SetI(kI);
    m_pidControllerLeft.SetD(kD);
    m_pidControllerLeft.SetIZone(kIz);
    m_pidControllerLeft.SetFF(kFF);
    m_pidControllerLeft.SetOutputRange(kMinOutput, kMaxOutput);
    m_pidControllerLeft.SetSmartMotionMaxVelocity(kMaxVel);
    m_pidControllerLeft.SetSmartMotionMinOutputVelocity(kMinVel);
    m_pidControllerLeft.SetSmartMotionMaxAccel(kMaxAcc);
    m_pidControllerLeft.SetSmartMotionAllowedClosedLoopError(kAllErr);


    m_pidControllerRight.SetP(kP);
    m_pidControllerRight.SetI(kI);
    m_pidControllerRight.SetD(kD);
    m_pidControllerRight.SetIZone(kIz);
    m_pidControllerRight.SetFF(kFF);
    m_pidControllerRight.SetOutputRange(kMinOutput, kMaxOutput);
    m_pidControllerRight.SetSmartMotionMaxVelocity(kMaxVel);
    m_pidControllerRight.SetSmartMotionMinOutputVelocity(kMinVel);
    m_pidControllerRight.SetSmartMotionMaxAccel(kMaxAcc);
    m_pidControllerRight.SetSmartMotionAllowedClosedLoopError(kAllErr);

    // 44 / 18 sprockets
    m_WristMotorEncoder.SetPositionConversionFactor( 1.0 / ( 25.0 ) );
    m_WristMotorEncoder.SetVelocityConversionFactor(  1.0 / ( 60 * 25.0 ) );
    m_WristPidController.SetP(kP2);
    m_WristPidController.SetI(kI2);
    m_WristPidController.SetD(kD2);
    m_WristPidController.SetIZone(kIz2);
    m_WristPidController.SetFF(kFF2);
    m_WristPidController.SetOutputRange(kMinOutput2, kMaxOutput2);
    m_WristPidController.SetSmartMotionMaxVelocity(kMaxVel2);
    m_WristPidController.SetSmartMotionMinOutputVelocity(kMinVel2);
    m_WristPidController.SetSmartMotionMaxAccel(kMaxAcc2);
    m_WristPidController.SetSmartMotionAllowedClosedLoopError(kAllErr2);


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

void Arm::initArm()
{
    m_ArmMotorRight.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_ArmMotorLeft.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_WristMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_ArmPosition = arm_position_t::HOLD_START_POSITION;
    m_ArmMotorLeftEncoder.SetPosition(  m_ArmEncoder.GetAbsolutePosition() );
    m_ArmMotorRightEncoder.SetPosition( m_ArmEncoder.GetAbsolutePosition() );
    m_WristMotorEncoder.SetPosition( m_WristEncoder.GetAbsolutePosition() );
    m_startArmAngle = m_ArmEncoder.GetAbsolutePosition();
    m_startWristAngle = m_WristEncoder.GetAbsolutePosition();

}

void Arm::disableArm()
{
    m_ArmMotorRight.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    m_ArmMotorLeft.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    m_WristMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
}

void Arm::SetArmPosition (arm_position_t DesiredPosition)
{
    m_ArmPosition = DesiredPosition;
}

void Arm::updateArm()
{
    double ArmAngle = 0;
    double WristAngle = 0;
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
    if((p != kP))   { m_pidControllerRight.SetP(p);m_pidControllerLeft.SetP(p); kP = p; }
    if((i != kI))   { m_pidControllerRight.SetI(i);m_pidControllerLeft.SetI(i); kI = i; }
    if((d != kD))   { m_pidControllerRight.SetD(d);m_pidControllerLeft.SetD(d); kD = d; }
    if((iz != kIz)) { m_pidControllerRight.SetIZone(iz);m_pidControllerLeft.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidControllerRight.SetFF(ff);m_pidControllerLeft.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { m_pidControllerRight.SetOutputRange(min, max);m_pidControllerLeft.SetOutputRange(min, max); kMinOutput = min; kMaxOutput = max; }
    if((maxV != kMaxVel)) { m_pidControllerRight.SetSmartMotionMaxVelocity(maxV);m_pidControllerLeft.SetSmartMotionMaxVelocity(maxV); kMaxVel = maxV; }
    if((minV != kMinVel)) { m_pidControllerRight.SetSmartMotionMinOutputVelocity(minV);m_pidControllerLeft.SetSmartMotionMinOutputVelocity(minV); kMinVel = minV; }
    if((maxA != kMaxAcc)) { m_pidControllerRight.SetSmartMotionMaxAccel(maxA);m_pidControllerLeft.SetSmartMotionMaxAccel(maxA); kMaxAcc = maxA; }
    if((allE != kAllErr)) { m_pidControllerRight.SetSmartMotionAllowedClosedLoopError(allE); m_pidControllerLeft.SetSmartMotionAllowedClosedLoopError(allE); allE = kAllErr; }
#endif

    switch (m_ArmPosition)
    {   
        case (HOLD_START_POSITION):
        {
            ArmAngle   = m_startArmAngle;
            WristAngle =  m_startWristAngle;
            break;
        }
        // TODO : Determine all arm and wrist positions
        case (GROUND_PICKUP):
        {
            ArmAngle   = 0.5099;
            WristAngle =  2.0;
            break;
        }
        case (SOURCE):
        {
            ArmAngle   = 0.4;
            WristAngle = 1.5;
            break;
        }
        case (SPEAKER):
        {
            ArmAngle   = 0.5099;
            WristAngle = 0.75;
            break;
        }
        case (AMP):
        {
            ArmAngle   = 0.3;
            WristAngle = 1.5;
            break;
        }
        case (TRAP):
        {
            ArmAngle   = 0.5099;
            WristAngle = 0.75;
            break;
        }
    }
    
    //PIDdly thing
    // const auto ArmControlOutput = m_ArmPIDController.Calculate(
    //     units::radian_t{m_ArmEncoder.GetDistance()}, units::radian_t{ArmAngle});

    // const auto WristControlOutput = m_WristPIDController.Calculate(
    //     units::radian_t{m_WristEncoder.GetDistance()}, units::radian_t{WristAngle});


   #if DBG_DISABLE_ARM_MOTORS
    m_ArmMotorLeft.Set( 0 );
    m_ArmMotorRight.Set( 0 );
   #else
    m_pidControllerLeft.SetReference(ArmAngle, rev::CANSparkMax::ControlType::kSmartMotion);
    m_pidControllerRight.SetReference(ArmAngle, rev::CANSparkMax::ControlType::kSmartMotion);
   #endif

   #if DBG_DISABLE_WRIST_MOTORS
    m_WristMotor.Set( 0 );
   #else
    m_WristPidController.SetReference(WristAngle, rev::CANSparkMax::ControlType::kSmartMotion);
    //m_WristMotor.Set(std::clamp( WristControlOutput, -0.1, 0.1 ));
   #endif

}

void Arm::UpdateSmartDashboardData()
{
#if 0
    frc::SmartDashboard::PutNumber("Arm_ControlOutputL",   m_ArmMotorLeft.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Arm_ControlOutputR",   m_ArmMotorRight.GetAppliedOutput());

    frc::SmartDashboard::PutNumber("Arm_Angle",           ArmAngle);
    frc::SmartDashboard::PutNumber("Arm_Position",        m_ArmPosition);
    frc::SmartDashboard::PutNumber("Arm_Encoder_Dist",    m_ArmEncoder.GetDistance());//Ground pickup:0.5099 Top Position 0.1844
    frc::SmartDashboard::PutNumber("Arm_Encoder_AbsPos",  m_ArmEncoder.GetAbsolutePosition());//Ground pickup:0.5099 Top Position 0.1844
    
    frc::SmartDashboard::PutNumber("Arm_NeoPositionL",     m_ArmMotorLeftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Arm_NeoPositionR",     m_ArmMotorRightEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Arm_NeoVelocityL",     m_ArmMotorLeftEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Arm_NeoVelocityR",     m_ArmMotorRightEncoder.GetVelocity());
#endif

    //frc::SmartDashboard::PutNumber("Wrist_ControlOutput", WristControlOutput);
    //frc::SmartDashboard::PutNumber("Wrist_Angle",         WristAngle);
    frc::SmartDashboard::PutNumber("Wrist_Position",      m_WristPosition);
    frc::SmartDashboard::PutNumber("Wrist_Encoder_Dist",    m_WristEncoder.GetDistance());//Shooting position:-1.3570 Ground Pickup:-0.0379
    frc::SmartDashboard::PutNumber("Wrist_Encoder_AbsPos",  m_WristEncoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("Wrist_MotorEncoder_Pos",  m_WristMotorEncoder.GetPosition());

    frc::SmartDashboard::PutNumber("Wrist_AppliedOutput",   m_WristMotor.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Wrist_NeoVelocity",     m_WristMotorEncoder.GetVelocity());
    //frc::SmartDashboard::PutNumber("Wrist_NeoVelocity",     m_WristPidController.Get());

}





void Arm::armManualControl( double speed )
{
    m_ArmMotorRight.Set(0.4 * speed);
    m_ArmMotorLeft.Set(0.4 * speed);
}


void Arm::wristManualControl( double speed )
{
    m_WristMotor.Set(0.2 * speed);
}


//Change to work for wrist???

//PID? PIN? Bowling? Wii Sports Bowling???
