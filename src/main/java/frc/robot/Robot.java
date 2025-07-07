// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Superstructure.SuperState;
import frc.robot.elevator.ElevatorIOReal;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.shoulder.ShoulderIOReal;
import frc.robot.shoulder.ShoulderSubsystem;
import frc.robot.wrist.WristIOReal;
import frc.robot.wrist.WristSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  public static enum ReefTarget {
    // L1(
    //     3.0,
    //     SuperState.SCORE_L1),
    L2(-15.0, SuperState.SCORE_L2),
    L3(-15.0, SuperState.SCORE_L3),
    L4(-20.0, SuperState.L4_TUCKED_OUT);

    public final double outtakeSpeed;
    public final SuperState state;

    private ReefTarget(double outtakeSpeed, SuperState state) {
      this.outtakeSpeed = outtakeSpeed;
      this.state = state;
    }
  }

  public static enum AlgaeIntakeTarget {
    LOW,
    HIGH,
    STACK,
    GROUND
  }

  public static enum AlgaeScoreTarget {
    NET,
    PROCESSOR
  }

  @AutoLogOutput private static ReefTarget currentCoralTarget = ReefTarget.L4;
  @AutoLogOutput private static AlgaeIntakeTarget algaeIntakeTarget = AlgaeIntakeTarget.STACK;
  @AutoLogOutput private static AlgaeScoreTarget algaeScoreTarget = AlgaeScoreTarget.NET;

  public static Trigger preScoreReq =
      new Trigger(
          () ->
              true); // TODO this would be the driver button
  public static Trigger scoreReq = new Trigger(() -> true);
  public static Trigger intakeAlgaeReq = new Trigger(() -> true);

  private Command m_autonomousCommand;

  private final ElevatorSubsystem elevator = new ElevatorSubsystem(new ElevatorIOReal());
  private final ShoulderSubsystem shoulder = new ShoulderSubsystem(new ShoulderIOReal());
  private final WristSubsystem wrist = new WristSubsystem(new WristIOReal());
  private final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();

  private final Superstructure superstructure =
      new Superstructure(
          elevator,
          shoulder,
          wrist,
          manipulator,
          () -> currentCoralTarget,
          () -> algaeIntakeTarget,
          () -> algaeScoreTarget);

  public Robot() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    superstructure.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
