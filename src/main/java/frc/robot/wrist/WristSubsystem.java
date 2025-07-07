// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {

  public static final double WRIST_GEAR_RATIO = 4.0 * 4.0 * (64.0 / 34.0);

  private Rotation2d setpoint = Rotation2d.kZero;

  public enum WristState {
    // INTAKE_ALGAE_GROUND(Rotation2d.fromDegrees(-65)),
    // INTAKE_ALGAE_STACK(Rotation2d.fromDegrees(-10)),
    // INTAKE_ALGAE_REEF(Rotation2d.fromDegrees(-20.0)),
    // READY_ALGAE,
    // PRE_NET,
    // SCORE_NET,
    PRE_INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(30.0)), // formerly WRIST_CLEARANCE_POS
    INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(0.0)),
    HP(Rotation2d.fromDegrees(178.0)),
    PRE_L2(Rotation2d.fromDegrees(170.0)),
    L2(Rotation2d.fromRadians(2.447)),
    PRE_L3(Rotation2d.fromDegrees(170.0)),
    L3(Rotation2d.fromRadians(2.427)),
    L4_TUCKED(Rotation2d.fromDegrees(170.0)), // WRIST_TUCKED_CLEARANCE_POS
    L4_TUCKED_OUT(Rotation2d.fromDegrees(120.0)),
    PRE_INTAKE_ALGAE_REEF(Rotation2d.fromDegrees(30.0)),
    INTAKE_ALGAE_REEF(Rotation2d.fromDegrees(-20.0)),
    READY_ALGAE_REEF(Rotation2d.fromDegrees(-10.0)),
    PRE_NET(Rotation2d.fromDegrees(100)),
    SCORE_NET(Rotation2d.fromDegrees(110))
  // SCORE_L1,
  // SCORE_L2,
  // SCORE_L3,
  // SCORE_L4,
  // WHACK_L1
  ;

    private final Rotation2d angle;

    private WristState(Rotation2d angle) {
      this.angle = angle;
    }

    public Rotation2d getAngle() {
      return angle;
    }
  }

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  /** Creates a new WristSubsystem. */
  public WristSubsystem(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Carriage/Wrist", inputs);
  }

  public Command setAngle(Supplier<Rotation2d> angle) {
    return this.run(
        () -> {
          io.setPosition(angle.get());
          setpoint = angle.get();
        });
  }

  public boolean isNearAngle(Rotation2d target) {
    return MathUtil.isNear(target.getDegrees(), inputs.motorPosition.getDegrees(), 10.0);
  }

  public boolean atSetpoint() {
    return isNearAngle(setpoint);
  }
}
