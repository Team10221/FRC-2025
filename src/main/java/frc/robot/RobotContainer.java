package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.AlgaeManipConstants.AlgaeManipState;
import frc.robot.Constants.CoralManipConstants.CoralManipState;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotState;
import frc.robot.auto.AutoModeSelector;

public class RobotContainer {
  public SwerveSubsystem swerve;
  public VisionSubsystem vision;
  public AlgaeManipulator algaeManipulator;
  public CoralManipulator coralManipulator;
  public Elevator elevator;

  public PS4Controller primary;
  public XboxController secondary;

  private RobotState currentState;
  private AutoModeSelector autoModeSelector;

  public RobotContainer() {
    try { this.swerve = new SwerveSubsystem(); }
    catch (Exception e) {
      System.err.println("YAGSL failed to read our drivetrain swerve configuration! D:");
      e.printStackTrace();
      System.exit(1);
    }
    this.vision = new VisionSubsystem(swerve);
    this.algaeManipulator = new AlgaeManipulator();
    this.coralManipulator = new CoralManipulator();
    this.elevator = new Elevator();

    this.primary = new PS4Controller(ControllerConstants.PRIMARY_PORT);
    this.secondary = new XboxController(ControllerConstants.SECONDARY_PORT);

    this.currentState = RobotState.NORMAL;
    this.autoModeSelector = new AutoModeSelector(this);
    
    configureBindings();

    swerve.setDefaultCommand(
      new RunCommand(
        () -> {
          // get speeds & apply deadbands
          double xSpeed = -MathUtil.applyDeadband(primary.getLeftY(), ControllerConstants.DEADBAND);
          double ySpeed = -MathUtil.applyDeadband(primary.getLeftX(), ControllerConstants.DEADBAND);
          double rotSpeed = -MathUtil.applyDeadband(primary.getRightX(), ControllerConstants.DEADBAND);

          // square speeds for better control
          xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
          ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
          rotSpeed = Math.copySign(rotSpeed * rotSpeed, rotSpeed);

          // apply multipliers
          double mul = currentState.driveSpeedMultiplier;

          xSpeed *= swerve.getMaxSpeed() * mul;
          ySpeed *= swerve.getMaxSpeed() * mul;
          rotSpeed *= ControllerConstants.ROTATION_SPEED * mul;

          swerve.drive(
            new Translation2d(xSpeed, ySpeed), 
            rotSpeed, 
            true 
          );
        },
        swerve
      )
    );
  }

  private void changeState(RobotState targetState) {
    if (currentState == RobotState.NORMAL) {
      // we're at the normal configuration - now we check what we want to score
      if (RobotConstants.ALGAE_STATES.contains(targetState)) {
        // in this case, we're transitioning to an algae state
        // firstly, the coral manipulator should move out of the way
        // afterward, the elevator and algae manipulator should change states
        if (targetState != RobotState.ALGAE_OUTTAKE_PROCESSOR) {
          new SequentialCommandGroup(
            new InstantCommand(() -> coralManipulator.setState(targetState.coralManipAngleState)),
            new WaitCommand(0.5),
            new InstantCommand(() -> elevator.setState(targetState.elevatorState)),
            new InstantCommand(() -> algaeManipulator.setState(targetState.algaeManipAngleState))
          ).schedule();
        } else {
          // in this case, we're transitioning from the normal state to the processor state
          // no need for a command then!
          elevator.setState(targetState.elevatorState);
          coralManipulator.setState(targetState.coralManipAngleState);
          algaeManipulator.setState(targetState.algaeManipAngleState);
        }
      } else {
        // in this case, we're transitioning to a coral state from normal
        // let's bring out the manipulator first, then the elevator
        new SequentialCommandGroup(
          new InstantCommand(() -> coralManipulator.setState(targetState.coralManipAngleState)),
          new WaitCommand(0.5),
          new InstantCommand(() -> elevator.setState(targetState.elevatorState))
        ).schedule();
      }
    } else if (RobotConstants.CORAL_STATES.contains(currentState)) {
      // in this case, the current state is a coral state
      // there are a few cases: -> algae, normal, coral
      if (RobotConstants.ALGAE_STATES.contains(targetState)) {
        // do nothing - we shouldn't be transitioning to algae from coral
      } else if (RobotConstants.CORAL_STATES.contains(targetState)) {
        // only thing that differs is elevator position
        elevator.setState(targetState.elevatorState);
      } else {
        // in this case, we're returning to normal from coral
        // first we bring the elevator down and bringing coral in
        new SequentialCommandGroup(
          new InstantCommand(() -> elevator.setState(targetState.elevatorState)),
          new WaitCommand(1),
          new InstantCommand(() -> coralManipulator.setState(targetState.coralManipAngleState))
        ).schedule();
      }
    } else if (RobotConstants.ALGAE_STATES.contains(currentState)) {
      // here, we're currently in an algae state
      // we shouldn't transition to coral from here, only to normal or another algae state
      if (RobotConstants.CORAL_STATES.contains(targetState)) {
        // do nothing
      } else if (RobotConstants.ALGAE_STATES.contains(targetState)) {
        if (targetState != RobotState.ALGAE_OUTTAKE_PROCESSOR) {
          elevator.setState(targetState.elevatorState);
        }
      } else {
        new SequentialCommandGroup(
          new InstantCommand(() -> elevator.setState(targetState.elevatorState)),
          new InstantCommand(() -> algaeManipulator.setState(targetState.algaeManipAngleState)),
          new WaitCommand(0.5),
          new InstantCommand(() -> coralManipulator.setState(targetState.coralManipAngleState)) 
        ).schedule();
      }
    }
    currentState = targetState;
  }

  private void configureBindings() {
    // PRIMARY CONTROLS
    new JoystickButton(primary, PS4Controller.Button.kCircle.value)
      .whileTrue(new RunCommand(() -> swerve.lockPose()));
    new JoystickButton(primary, PS4Controller.Button.kTriangle.value)
      .onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    new JoystickButton(primary, PS4Controller.Button.kL1.value)
      .whileTrue(new RunCommand(() -> algaeManipulator.setState(AlgaeManipState.INTAKE)))
      .onFalse(new InstantCommand(() -> algaeManipulator.setState(AlgaeManipState.IDLE)));
    new JoystickButton(primary, PS4Controller.Button.kL2.value)
      .whileTrue(new RunCommand(() -> algaeManipulator.setState(AlgaeManipState.RELEASE)))
      .onFalse(new InstantCommand(() -> algaeManipulator.setState(AlgaeManipState.IDLE)));
    new JoystickButton(primary, PS4Controller.Button.kR1.value)
      .whileTrue(new RunCommand(() -> algaeManipulator.setState(CoralManipState.INTAKE)))
      .onFalse(new InstantCommand(() -> algaeManipulator.setState(CoralManipState.REST)));
    new JoystickButton(primary, PS4Controller.Button.kR2.value)
      .whileTrue(new RunCommand(() -> algaeManipulator.setState(CoralManipState.OUTTAKE)))
      .onFalse(new InstantCommand(() -> algaeManipulator.setState(CoralManipState.REST))); 

    // SECONDARY CONTROLS
    new JoystickButton(secondary, XboxController.Button.kX.value)
      .onTrue(new InstantCommand(() -> changeState(RobotState.NORMAL)));
    new JoystickButton(secondary, XboxController.Button.kY.value)
      .onTrue(new InstantCommand(() -> changeState(RobotState.ALGAE_OUTTAKE_PROCESSOR)));
    new JoystickButton(secondary, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(() -> changeState(RobotState.ALGAE_REEF_INTAKE_L2)));
    new JoystickButton(secondary, XboxController.Button.kB.value)
      .onTrue(new InstantCommand(() -> changeState(RobotState.ALGAE_REEF_INTAKE_L3)));
    new POVButton(secondary, 0)
      .onTrue(new InstantCommand(() -> changeState(RobotState.CORAL_L2)));
    new POVButton(secondary, 90)
      .onTrue(new InstantCommand(() -> changeState(RobotState.CORAL_L3)));
    new POVButton(secondary, 180)
      .onTrue(new InstantCommand(() -> changeState(RobotState.CORAL_L4)));
  }

  public Command getAutonomousCommand() {
    return autoModeSelector.getAutonomousCommand();
  }
}
