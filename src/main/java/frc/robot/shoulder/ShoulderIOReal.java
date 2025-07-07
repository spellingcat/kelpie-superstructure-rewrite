// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shoulder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ShoulderIOReal implements ShoulderIO {
  private final TalonFX motor;
  private final CANcoder cancoder;

  private final StatusSignal<Angle> motorPositionRotations;
  private final StatusSignal<Angle> cancoderPositionRotations;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<AngularVelocity> angularVelocityRPS;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicTorqueCurrentFOC positionTorque =
      new MotionMagicTorqueCurrentFOC(0.0); // TODO torque current or duty cycle

  public ShoulderIOReal() {
    motor = new TalonFX(0, "*");
    cancoder = new CANcoder(0, "*");

    motorPositionRotations = motor.getPosition();
    cancoderPositionRotations = cancoder.getAbsolutePosition();
    appliedVoltage = motor.getMotorVoltage();
    angularVelocityRPS = motor.getVelocity();

    final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    // TODO PID things should go here
    // and other configs

    final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    cancoder.getConfigurator().apply(cancoderConfig);
    motor.getConfigurator().apply(motorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        motorPositionRotations,
        cancoderPositionRotations,
        appliedVoltage,
        angularVelocityRPS);
  }

  @Override
  public void updateInputs(ShoulderIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorPositionRotations, cancoderPositionRotations, appliedVoltage, angularVelocityRPS);

    inputs.motorPosition = Rotation2d.fromRotations(motorPositionRotations.getValueAsDouble());
    inputs.cancoderPosition =
        Rotation2d.fromRotations(cancoderPositionRotations.getValueAsDouble());
    inputs.voltage = appliedVoltage.getValueAsDouble();
    inputs.angularVelocityRPS = angularVelocityRPS.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setMotorPosition(Rotation2d position) {
    motor.setControl(positionTorque.withPosition(position.getRotations()));
  }

  @Override
  public void setEncoderPosition(Rotation2d position) {
    cancoder.setPosition(position.getRotations());
  }
}
