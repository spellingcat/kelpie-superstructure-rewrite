// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shoulder;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShoulderSubsystem extends SubsystemBase {

  public static final double SHOULDER_FINAL_STAGE_RATIO = 3.0;
  public static final double SHOULDER_GEAR_RATIO =
      25.0 * (34.0 / 28.0) * SHOULDER_FINAL_STAGE_RATIO;
  public static final int CANCODER_ID = 5;
  public static final Rotation2d MAX_SHOULDER_ROTATION = Rotation2d.fromDegrees(120.0);
  public static final Rotation2d MIN_SHOULDER_ROTATION = Rotation2d.fromDegrees(-5.0);

  public static final double X_OFFSET_METERS = 0.1016254;
  public static final double Z_OFFSET_METERS = 0.207645;
  public static final double ARM_LENGTH_METERS = Units.inchesToMeters(13.5);

  public static final MotionMagicConfigs TOSS_CONFIGS =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(0.275)
          .withMotionMagicAcceleration(4.0);

  private Rotation2d setpoint = Rotation2d.kZero;

  public enum ShoulderState {
    HP(Rotation2d.fromDegrees(50.0)),
    PRE_INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(35.0)),
    INTAKE_CORAL_GROUND(Rotation2d.fromDegrees(8.0)),
    PRE_L2(Rotation2d.fromDegrees(35.0)),
    L2(Rotation2d.fromRadians(0.569).plus(Rotation2d.fromDegrees(20))),
    PRE_L3(Rotation2d.fromDegrees(35.0)),
    L3(Rotation2d.fromRadians(1.022).minus(Rotation2d.fromDegrees(3))),
    L4_TUCKED(Rotation2d.fromDegrees(35.0)), // SHOULDER_TUCKED_CLEARANCE_POS
    L4_TUCKED_OUT(Rotation2d.fromDegrees(25.0)),
    PRE_INTAKE_ALGAE_REEF(Rotation2d.fromDegrees(35.0)),
    INTAKE_ALGAE_REEF(Rotation2d.fromDegrees(45.0)),
    READY_ALGAE_REEF(Rotation2d.fromDegrees(90.0)),
    PRE_NET(Rotation2d.fromDegrees(30)),
    SCORE_NET(Rotation2d.fromDegrees(90));

    private final Rotation2d angle;

    private ShoulderState(Rotation2d angle) {
      this.angle = angle;
    }

    public Rotation2d getAngle() {
      return angle;
    }
  }

  private final ShoulderIO io;
  private final ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();

  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem(ShoulderIO io) {
    this.io = io;
    rezero();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Carriage/Shoulder", inputs);
  }

  @AutoLogOutput(key = "Shoulder/Zeroing Angle")
  public Rotation2d getZeroingAngle() {
    return inputs.cancoderPosition.div(SHOULDER_FINAL_STAGE_RATIO); // TODO
  }

  public void rezero() {
    io.setEncoderPosition(getZeroingAngle());
  }

  public Command setAngle(Supplier<Rotation2d> target) {
    return this.run(
        () -> {
          io.setMotorPosition(target.get());
          setpoint = target.get();
        });
  }

  public Command hold() {
    return Commands.sequence(
        setAngle(() -> inputs.motorPosition).until(() -> true), this.run(() -> {}));
  }

  public boolean isNearAngle(Rotation2d target) {
    return MathUtil.isNear(target.getDegrees(), inputs.motorPosition.getDegrees(), 10.0);
  }

  public boolean atSetpoint() {
    return isNearAngle(setpoint);
  }

  public double getVelocity() {
    return inputs.angularVelocityRPS;
  }
}
