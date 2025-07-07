// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

  public static final double GEAR_RATIO = 2.5 / 1.0;
  public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.751 / 2.0);
  public static final Rotation2d ELEVATOR_ANGLE = Rotation2d.fromDegrees(90.0);
  public static final double X_OFFSET_METERS = Units.inchesToMeters(4.0);

  /** Offset from origin to center of pivot */
  public static final double Z_OFFSET_METERS = Units.inchesToMeters(8.175000);

  public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(63.50);

  public static final double MAX_ACCELERATION = 10.0;
  public static final double SLOW_ACCELERATION = 5.0;
  public static final double MEDIUM_ACCELERATION = 8.5;

  private double setpoint = 0.0;

  public enum ElevatorState {
    HP(Units.inchesToMeters(0.0)),
    INTAKE_CORAL_GROUND(Units.inchesToMeters(0.0)),
    L2(0.23 + Units.inchesToMeters(1.5)),
    L3(0.60 + Units.inchesToMeters(2.0)),
    L4(MAX_EXTENSION_METERS), // TODO why can't i find this number lol??
    INTAKE_ALGAE_HIGH(Units.inchesToMeters(35.0)),
    NET(Units.inchesToMeters(62.5));

    private final double extensionMeters;

    private ElevatorState(double extensionMeters) {
      this.extensionMeters = extensionMeters;
    }

    public double getExtensionMeters() {
      return extensionMeters;
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public Command setPosition(DoubleSupplier positionMeters) {
    return this.run(
        () -> {
          io.setPosition(positionMeters.getAsDouble());
          setpoint = positionMeters.getAsDouble();
        });
  }

  public Command hold() {
    return Commands.sequence(
        setPosition(() -> inputs.positionMeters).until(() -> true), this.run(() -> {}));
  }

  public Command holdPosition(DoubleSupplier positionMeters) {
    return Commands.sequence(setPosition(positionMeters).until(() -> true), this.run(() -> {}));
  }

  public boolean atExtension(double expected) {
    return MathUtil.isNear(expected, inputs.positionMeters, 0.05);
  }

  public boolean atSetpoint() {
    return atExtension(setpoint);
  }
}
