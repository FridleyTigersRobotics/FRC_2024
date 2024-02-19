// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>
//#include <Cameron's Brain> [ERROR:NONEXISTANT] HAHA FUNNY
#include <frc/geometry/Rotation2d.h>
#include <string>
#include <fmt/printf.h>
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel,
                           const int turningEncoderChannel,
                           const double turningEncoderOffset)
    : m_driveMotor(driveMotorChannel,rev::CANSparkLowLevel::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel,rev::CANSparkLowLevel::MotorType::kBrushless),

      
      m_turningEncoder(turningEncoderChannel) {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveEncoder.SetPosition(0); 
  double positonConversionFactor = 3.9*std::numbers::pi/8.14*0.0254;
  m_PositionConversionFactor = positonConversionFactor;
  m_driveEncoder.SetPositionConversionFactor(positonConversionFactor);
  m_driveEncoder.SetVelocityConversionFactor((1.0/60.0) * positonConversionFactor);

  m_drivechannel=driveMotorChannel;
  //m_encodername=fmt::sprintf("turnencoder %d",m_drivechannel);


  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.
  m_turningEncoder.SetDistancePerRotation(2 * std::numbers::pi);
  m_turningEncoder.SetPositionOffset(turningEncoderOffset);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      -units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          units::radian_t{m_turningEncoder.GetDistance()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {units::meter_t{m_driveEncoder.GetPosition()},
          units::radian_t{m_turningEncoder.GetDistance()}};
}



void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {

  frc::Rotation2d encoderRotation{
      units::radian_t{m_turningEncoder.GetDistance()}};

  // Optimize the reference state to avoid spinning further than 90 degrees
  auto state =
      frc::SwerveModuleState::Optimize(referenceState, encoderRotation);

  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  state.speed *= (state.angle - encoderRotation).Cos();

  // Calculate the drive output from the drive PID controller.

  const auto posConvFact = m_PositionConversionFactor; //m_driveEncoder.GetPositionConversionFactor();

  const auto stateSpeedVal = state.speed.value();

  const auto driveOutput = m_drivePIDController.Calculate(
      posConvFact, stateSpeedVal);

  const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  const auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{m_turningEncoder.GetDistance()}, state.angle.Radians());

  const auto turnFeedforward = units::volt_t{0.0};/*m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);*/

  // Set the motor outputs.
  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}