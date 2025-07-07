// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface WristIO {

  @AutoLog
  public class WristIOInputs {
    public Rotation2d motorPosition = new Rotation2d();
  }

  public void updateInputs(WristIOInputs inputs);

  public void setPosition(Rotation2d position);
}
