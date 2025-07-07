// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

/** Add your docs here. */
public class WristIOReal implements WristIO {
  private final TalonFX motor;
  private final StatusSignal<Angle> position;

  private final MotionMagicTorqueCurrentFOC positionTorque =
      new MotionMagicTorqueCurrentFOC(
          0.0); // TODO i still don't know which control request you would use when

  public WristIOReal() {
    motor = new TalonFX(0, "*");

    position = motor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position);
  }

  @Override
  public void setPosition(Rotation2d position) {
    motor.setControl(positionTorque.withPosition(position.getRotations()));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    BaseStatusSignal.refreshAll(position);

    inputs.motorPosition = Rotation2d.fromRotations(position.getValueAsDouble());
  }
}
