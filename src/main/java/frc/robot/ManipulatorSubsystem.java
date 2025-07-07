// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class ManipulatorSubsystem extends SubsystemBase {

  public static final double JOG_POS = 0.75;
  public static final double ALGAE_HOLDING_VOLTAGE = 1.0;
  public static final double ALGAE_INTAKE_VOLTAGE = 10.0;

  /** Not implemented yet, i just need the methods for now */
  public ManipulatorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command hold() {
    return Commands.none();
  }

  public Command jog(double rotations) {
    return Commands.none();
  }

  public Command setVelocity(DoubleSupplier vel) {
    return Commands.none();
  }

  public boolean getFirstBeambreak() {
    return true;
  }

  public boolean getSecondBeambreak() {
    return true;
  }

  public double getTimeSinceZero() {
    return 0.0;
  }

  public Command setVoltage(double volts) {
    return Commands.none();
  }

  public Command intakeAlgae() {
    return Commands.none();
  }
}
