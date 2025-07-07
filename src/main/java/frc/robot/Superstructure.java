// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.AlgaeIntakeTarget;
import frc.robot.Robot.AlgaeScoreTarget;
import frc.robot.Robot.ReefTarget;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.shoulder.ShoulderSubsystem;
import frc.robot.shoulder.ShoulderSubsystem.ShoulderState;
import frc.robot.wrist.WristSubsystem;
import frc.robot.wrist.WristSubsystem.WristState;
import java.util.ArrayList;
import java.util.function.Supplier;

public class Superstructure {
  public enum SuperState {
    IDLE(ElevatorState.HP, ShoulderState.HP, WristState.HP),
    // HOME(),
    PRE_INTAKE_CORAL_GROUND(
        ElevatorState.INTAKE_CORAL_GROUND,
        ShoulderState.PRE_INTAKE_CORAL_GROUND,
        WristState.PRE_INTAKE_CORAL_GROUND),
    INTAKE_CORAL_GROUND(
        ElevatorState.INTAKE_CORAL_GROUND,
        ShoulderState.INTAKE_CORAL_GROUND,
        WristState.INTAKE_CORAL_GROUND),
    READY_CORAL(ElevatorState.HP, ShoulderState.HP, WristState.HP),
    // SPIT_CORAL(),
    // PRE_L1(

    // ),
    PRE_L2(ElevatorState.L2, ShoulderState.PRE_L2, WristState.PRE_L2),
    SCORE_L2(ElevatorState.L2, ShoulderState.L2, WristState.L2),
    POST_L2(
        ElevatorState.L2,
        ShoulderState.PRE_L2,
        WristState.PRE_L2),
    PRE_L3(ElevatorState.L3, ShoulderState.PRE_L3, WristState.PRE_L3),
    SCORE_L3(ElevatorState.L3, ShoulderState.L3, WristState.L3),
    POST_L3(ElevatorState.L3, ShoulderState.PRE_L3, WristState.PRE_L3),
    L4_TUCKED(ElevatorState.HP, ShoulderState.L4_TUCKED, WristState.L4_TUCKED),
    L4_TUCKED_EXTENDED(ElevatorState.L4, ShoulderState.L4_TUCKED, WristState.L4_TUCKED),
    L4_TUCKED_OUT(ElevatorState.L4, ShoulderState.L4_TUCKED_OUT, WristState.L4_TUCKED_OUT),
    // ANTI_JAM,
    // INTAKE_ALGAE_GROUND,
    PRE_PRE_INTAKE_ALGAE_HIGH(
        ElevatorState.HP, ShoulderState.PRE_INTAKE_ALGAE_REEF, WristState.PRE_INTAKE_ALGAE_REEF),
    PRE_INTAKE_ALGAE_HIGH(
        ElevatorState.HP, ShoulderState.INTAKE_ALGAE_REEF, WristState.INTAKE_ALGAE_REEF),
    INTAKE_ALGAE_HIGH(
        ElevatorState.INTAKE_ALGAE_HIGH,
        ShoulderState.INTAKE_ALGAE_REEF,
        WristState.INTAKE_ALGAE_REEF),
    HOLD_ALGAE(
        ElevatorState.INTAKE_ALGAE_HIGH,
        ShoulderState.READY_ALGAE_REEF,
        WristState.READY_ALGAE_REEF),
    PRE_NET(ElevatorState.NET, ShoulderState.PRE_NET, WristState.PRE_NET),
    SCORE_NET(ElevatorState.NET, ShoulderState.SCORE_NET, WristState.SCORE_NET)
  // INTAKE_ALGAE_LOW,
  // INTAKE_ALGAE_STACK,
  // SPIT_ALGAE,
  // PRE_PROCESSOR,
  // SCORE_ALGAE_PROCESSOR,
  // PRE_CLIMB,
  // CLIMB
  ;

    // A better way to do this is probably to actually treat each subsystem like its own state
    // machine--each subsystem would respond to triggers on its own in its own file--and I suspect
    // that will be a better approach with the
    // clone, but I think each subsystem's movements on Kelpie are too interdependent for scattering
    // the logic across different files to make sense or for it to not be overcomplicated
    // So these aren't *truly* states, but hopefully will be going forward
    public final ElevatorState elevatorState;
    public final ShoulderState shoulderState;
    public final WristState wristState;

    private SuperState(
        ElevatorState elevatorState, ShoulderState shoulderState, WristState wristState) {
      this.elevatorState = elevatorState;
      this.shoulderState = shoulderState;
      this.wristState = wristState;
    }
  }

  /**
   * @param start first state
   * @param end second state
   * @param trigger trigger to make it go from the first state to the second (assuming it's already
   *     in the first state)
   * @param cmd cmd to make it go from the first state to the second
   */
  public record Transition(SuperState start, SuperState end, Trigger trigger, Command cmd) {}
  ;

  private SuperState state = SuperState.IDLE;
  private SuperState prevState = SuperState.IDLE;

  private ArrayList<Transition> transitions = new ArrayList<>();

  private Timer stateTimer = new Timer();

  private final Supplier<ReefTarget> reefTarget;
  private final Supplier<AlgaeIntakeTarget> algaeIntakeTarget;
  private final Supplier<AlgaeScoreTarget> algaeScoreTarget;

  private final ElevatorSubsystem elevator;
  private final ShoulderSubsystem shoulder;
  private final WristSubsystem wrist;
  private final ManipulatorSubsystem manipulator;

  /** Creates a new Superstructure. */
  public Superstructure(
      ElevatorSubsystem elevator,
      ShoulderSubsystem shoulder,
      WristSubsystem wrist,
      ManipulatorSubsystem manipulator,
      Supplier<ReefTarget> reefTarget,
      Supplier<AlgaeIntakeTarget> algaeIntakeTarget,
      Supplier<AlgaeScoreTarget> algaeScoreTarget) {
    this.elevator = elevator;
    this.shoulder = shoulder;
    this.wrist = wrist;
    this.manipulator = manipulator;

    this.reefTarget = reefTarget;
    this.algaeIntakeTarget = algaeIntakeTarget;
    this.algaeScoreTarget = algaeScoreTarget;

    addTransitions();
  }

  public void periodic() {
    for (Transition t : transitions) {
      if (state == t.start && t.trigger.getAsBoolean()) {
        forceState(t.end);
        t.cmd();
        return;
      }
    }
  }

  public void addTransitions() {

    transitions.add(
        new Transition(
            SuperState.IDLE,
            SuperState.READY_CORAL,
            new Trigger(
                () -> manipulator.getFirstBeambreak() && manipulator.getTimeSinceZero() < 1.0),
            this.holdExtension(SuperState.READY_CORAL)));

    // ------L2------
    transitions.add(
        new Transition(
            SuperState.READY_CORAL,
            SuperState.PRE_L2,
            Robot.preScoreReq.and(() -> reefTarget.get() == ReefTarget.L2),
            Commands.parallel(
                this.holdExtension(SuperState.PRE_L2),
                manipulator
                    .hold()
                    .until(
                        () ->
                            shoulder.isNearAngle(SuperState.PRE_L2.shoulderState.getAngle())
                                && wrist.isNearAngle(SuperState.PRE_L2.wristState.getAngle()))
                    .andThen(manipulator.jog(ManipulatorSubsystem.JOG_POS)))));

    transitions.add(
        new Transition(
            SuperState.PRE_L2,
            SuperState.SCORE_L2,
            Robot.scoreReq,
            Commands.parallel(
                this.holdExtension(SuperState.SCORE_L2),
                manipulator
                    .hold()
                    .until(
                        () -> elevator.atSetpoint() && shoulder.atSetpoint() && wrist.atSetpoint())
                    .andThen(manipulator.setVelocity(() -> reefTarget.get().outtakeSpeed)))));

    transitions.add(
        new Transition(
            SuperState.SCORE_L2,
            SuperState.POST_L2,
            new Trigger(
                () ->
                    !manipulator.getFirstBeambreak()
                        && !manipulator
                            .getSecondBeambreak()), // TODO and no algae req and away from reef
            this.holdExtension(SuperState.POST_L2)));

    transitions.add(
        new Transition(
            SuperState.POST_L2,
            SuperState.IDLE,
            new Trigger(() -> this.atExtension(SuperState.POST_L2)),
            idleCommand()));

    // ------L3------
    transitions.add(
        new Transition(
            SuperState.READY_CORAL,
            SuperState.PRE_L3,
            Robot.preScoreReq.and(() -> reefTarget.get() == ReefTarget.L3),
            Commands.parallel(
                this.holdExtension(SuperState.PRE_L3),
                manipulator
                    .hold()
                    .until(
                        () ->
                            shoulder.isNearAngle(SuperState.PRE_L3.shoulderState.getAngle())
                                && wrist.isNearAngle(SuperState.PRE_L3.wristState.getAngle()))
                    .andThen(manipulator.jog(ManipulatorSubsystem.JOG_POS)))));

    transitions.add(
        new Transition(
            SuperState.PRE_L3,
            SuperState.SCORE_L3,
            Robot.scoreReq,
            Commands.parallel(
                this.holdExtension(SuperState.SCORE_L3),
                manipulator
                    .hold()
                    .until(
                        () -> elevator.atSetpoint() && shoulder.atSetpoint() && wrist.atSetpoint())
                    .andThen(manipulator.setVelocity(() -> reefTarget.get().outtakeSpeed)))));

    transitions.add(
        new Transition(
            SuperState.SCORE_L3,
            SuperState.POST_L3,
            new Trigger(
                () ->
                    !manipulator.getFirstBeambreak()
                        && !manipulator
                            .getSecondBeambreak()), // TODO and no algae req and away from reef
            this.holdExtension(SuperState.POST_L3)));

    transitions.add(
        new Transition(
            SuperState.POST_L3,
            SuperState.IDLE,
            new Trigger(() -> this.atExtension(SuperState.POST_L3)),
            idleCommand()));

    // ------L4------
    transitions.add(
        new Transition(
            SuperState.READY_CORAL,
            SuperState.L4_TUCKED,
            Robot.preScoreReq.and(() -> reefTarget.get() == ReefTarget.L4),
            this.holdExtension(SuperState.L4_TUCKED)));

    transitions.add(
        new Transition(
            SuperState.L4_TUCKED,
            SuperState.L4_TUCKED_EXTENDED,
            Robot.preScoreReq
                .and(() -> reefTarget.get() == ReefTarget.L4)
                .and(
                    new Trigger(
                        () ->
                            this.atExtension(
                                SuperState
                                    .L4_TUCKED))), // TODO not sure if i need this to check for
            // prescore again?
            this.holdExtension(SuperState.L4_TUCKED_EXTENDED)));

    transitions.add(
        new Transition(
            SuperState.L4_TUCKED_EXTENDED,
            SuperState.L4_TUCKED_OUT,
            Robot.scoreReq
                .and(() -> reefTarget.get() == ReefTarget.L4)
                .and(
                    new Trigger(
                        () ->
                            this.atExtension(
                                SuperState
                                    .L4_TUCKED_EXTENDED))), // TODO not sure if i need this to check
            // for prescore again?
            Commands.parallel(
                this.holdExtension(SuperState.L4_TUCKED_OUT),
                manipulator
                    .hold()
                    .until(
                        () -> elevator.atSetpoint() && shoulder.atSetpoint() && wrist.atSetpoint())
                    .andThen(manipulator.setVelocity(() -> reefTarget.get().outtakeSpeed)))));

    transitions.add(
        new Transition(
            SuperState.L4_TUCKED_OUT,
            SuperState.L4_TUCKED_EXTENDED,
            new Trigger(
                () ->
                    !manipulator.getFirstBeambreak()
                        && !manipulator
                            .getSecondBeambreak()), // TODO and no algae req and away from reef
            Commands.parallel(this.holdExtension(SuperState.L4_TUCKED_EXTENDED))));

    transitions.add(
        new Transition(
            SuperState.L4_TUCKED_EXTENDED,
            SuperState.IDLE,
            new Trigger(() -> this.atExtension(SuperState.L4_TUCKED_EXTENDED)),
            idleCommand()));

    // ------Algae------
    transitions.add(
        new Transition(
            SuperState.IDLE,
            SuperState.PRE_PRE_INTAKE_ALGAE_HIGH,
            new Trigger(
                () ->
                    Robot.intakeAlgaeReq.getAsBoolean()
                        && algaeIntakeTarget.get() == AlgaeIntakeTarget.HIGH),
            Commands.parallel(
                this.holdExtension(SuperState.PRE_PRE_INTAKE_ALGAE_HIGH),
                manipulator.setVoltage(ManipulatorSubsystem.ALGAE_INTAKE_VOLTAGE))));

    transitions.add(
        new Transition(
            SuperState.PRE_PRE_INTAKE_ALGAE_HIGH,
            SuperState.PRE_INTAKE_ALGAE_HIGH,
            new Trigger(() -> atExtension(SuperState.PRE_PRE_INTAKE_ALGAE_HIGH)),
            Commands.parallel(
                holdExtension(SuperState.PRE_INTAKE_ALGAE_HIGH),
                manipulator.setVoltage(ManipulatorSubsystem.ALGAE_INTAKE_VOLTAGE))));

    transitions.add(
        new Transition(
            SuperState.PRE_INTAKE_ALGAE_HIGH,
            SuperState.INTAKE_ALGAE_HIGH,
            new Trigger(() -> atExtension(SuperState.PRE_INTAKE_ALGAE_HIGH)),
            Commands.parallel(
                holdExtension(SuperState.INTAKE_ALGAE_HIGH), manipulator.intakeAlgae())));

    transitions.add(
        new Transition(
            SuperState.INTAKE_ALGAE_HIGH,
            SuperState.HOLD_ALGAE,
            new Trigger(
                () -> true // TODO
                //   stateTimer.hasElapsed(1.0) &&
                //   manipulator.getStatorCurrentAmps() >
                // ManipulatorSubsystem.ALGAE_CURRENT_THRESHOLD || Robot.ROBOT_TYPE == RobotType.SIM
                // &&
                //   AlgaeIntakeTargets.getClosestTargetPose(pose.get())
                //                     .getTranslation()
                //                     .minus(pose.get().getTranslation())
                //                     .getNorm()
                //                 > 0.3
                ),
            this.holdExtension(SuperState.HOLD_ALGAE)));

    transitions.add(
        new Transition(
            SuperState.HOLD_ALGAE,
            SuperState.PRE_NET,
            Robot.preScoreReq.and(() -> algaeScoreTarget.get() == AlgaeScoreTarget.NET),
            Commands.parallel(
                manipulator.setVoltage(3 * ManipulatorSubsystem.ALGAE_HOLDING_VOLTAGE),
                this.holdExtension(SuperState.PRE_NET)) // TODO should be slowed
            ));

    transitions.add(
        new Transition(
            SuperState.PRE_NET,
            SuperState.SCORE_NET,
            new Trigger(() -> this.atExtension(SuperState.PRE_NET)),
            Commands.sequence(
                Commands.runOnce(() -> stateTimer.reset()),
                Commands.parallel(
                    manipulator
                        .hold()
                        .until(
                            () ->
                                shoulder.getVelocity()
                                    > ShoulderSubsystem.TOSS_CONFIGS.MotionMagicCruiseVelocity
                                        - 0.1)
                        .andThen(manipulator.setVoltage(-13.0)),
                    this.holdExtension(SuperState.SCORE_NET)))));

    transitions.add(
        new Transition(
            SuperState.SCORE_NET,
            SuperState.PRE_NET, // i realize PRE_NET is a misnomer
            new Trigger(
                () -> stateTimer.hasElapsed(0.5)), // TODO i don't trust this state timer stuff
            this.holdExtension(SuperState.PRE_NET)));

    transitions.add(
        new Transition(
            SuperState.PRE_NET,
            SuperState.IDLE,
            new Trigger(() -> stateTimer.hasElapsed(1.0)),
            this.idleCommand()));
  }

  private Command idleCommand() {
    return Commands.parallel(
        this.holdExtension(SuperState.IDLE).repeatedly()
        // ,
        // manipulator.intakeCoralAir(-7.0).repeatedly(),
        // funnel.setVoltage(
        //           () ->
        //               revFunnelReq.getAsBoolean()
        //                   ? -2.0
        //                   : (forceFunnelReq.getAsBoolean()
        //                           || (Stream.of(HumanPlayerTargets.values())
        //                                   .map(
        //                                       (t) ->
        //                                           t.location
        //                                               .minus(pose.get())
        //                                               .getTranslation()
        //                                               .getNorm())
        //                                   .min(Double::compare)
        //                                   .get()
        //                               < 1.0)
        //                       ? 1.0
        //                       : 0.0))
        );
  }

  private Command holdExtension(SuperState state) {
    return Commands.parallel(
        elevator.holdPosition(() -> state.elevatorState.getExtensionMeters()),
        shoulder.setAngle(() -> state.shoulderState.getAngle()),
        wrist.setAngle(() -> state.wristState.getAngle()));
  }

  private boolean atExtension(SuperState state) {
    return elevator.atExtension(state.elevatorState.getExtensionMeters())
        && shoulder.isNearAngle(state.shoulderState.getAngle())
        && wrist.isNearAngle(state.wristState.getAngle());
  }

  private Command forceState(SuperState nextState) {
    return Commands.runOnce(
            () -> {
              System.out.println("Changing state to " + nextState);
              stateTimer.reset();
              this.prevState = this.state;
              this.state = nextState;
            })
        .ignoringDisable(true);
  }
}
