package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.RobotState;
import frc.robot.Constants.AlgaeManipConstants.AlgaeManipState;
import frc.robot.Constants.CoralManipConstants.CoralManipState;
import frc.robot.other.RobotStateManager;

public class RobotContainer {
  public SwerveSubsystem swerve;
  public VisionSubsystem vision;
  public Elevator elevator;
  public AlgaeManipulator algaeManipulator;
  public CoralManipulator coralManipulator;
  public RobotStateManager stateManager;

  public XboxController primary;
  public XboxController secondary;

  // private AutoModeSelector autoModeSelector;

  public RobotContainer() {
    try { this.swerve = new SwerveSubsystem(); }
    catch (Exception e) {
      System.err.println("YAGSL failed to read our drivetrain swerve configuration! D:");
      e.printStackTrace();
      System.exit(1);
    }

    this.vision = new VisionSubsystem(swerve);
    this.elevator = new Elevator();
    this.coralManipulator = new CoralManipulator();
    this.algaeManipulator = new AlgaeManipulator();

    this.stateManager = new RobotStateManager(elevator, algaeManipulator, coralManipulator);

    this.primary = new XboxController(ControllerConstants.PRIMARY_PORT);
    this.secondary = new XboxController(ControllerConstants.SECONDARY_PORT);

    // this.autoModeSelector = new AutoModeSelector(this);
    
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
          double mul = stateManager.getCurrentState().driveSpeedMultiplier;

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

  private void configureBindings() {
    // PRIMARY CONTROLS

    new JoystickButton(primary, XboxController.Button.kLeftBumper.value)
      .whileTrue(new RunCommand(() -> swerve.lockPose()));
    new JoystickButton(primary, XboxController.Button.kRightBumper.value)
      .onTrue(new InstantCommand(() -> swerve.zeroGyro()));

    new JoystickButton(primary, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(() -> coralManipulator.setState(CoralManipState.INTAKE)))
      .onFalse(new InstantCommand(() -> coralManipulator.setState(CoralManipState.REST)));
    new JoystickButton(primary, XboxController.Button.kB.value)
      .onTrue(new InstantCommand(() -> coralManipulator.setState(CoralManipState.OUTTAKE)))
      .onFalse(new InstantCommand(() -> coralManipulator.setState(CoralManipState.REST)));

    /*
    new JoystickButton(primary, XboxController.Button.kX.value)
      .onTrue(new InstantCommand(() -> algaeManipulator.setState(AlgaeManipState.INTAKE)))
      .onFalse(new InstantCommand(() -> algaeManipulator.setState(AlgaeManipState.IDLE)));
    new JoystickButton(primary, XboxController.Button.kY.value)
      .onTrue(new InstantCommand(() -> algaeManipulator.setState(AlgaeManipState.RELEASE)))
      .onFalse(new InstantCommand(() -> algaeManipulator.setState(AlgaeManipState.IDLE)));
    */

    // SECONDARY CONTROLS
    
    new JoystickButton(secondary, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(() -> stateManager.changeState(RobotState.DEFAULT)));
    new JoystickButton(secondary, XboxController.Button.kB.value)
      .onTrue(new InstantCommand(() -> stateManager.changeState(RobotState.CORAL_L2)));
    new JoystickButton(secondary, XboxController.Button.kX.value)
      .onTrue(new InstantCommand(() -> stateManager.changeState(RobotState.CORAL_L3)));
    new JoystickButton(secondary, XboxController.Button.kY.value)
      .onTrue(new InstantCommand(() -> stateManager.changeState(RobotState.CORAL_L4)));

    new Trigger(() -> secondary.getPOV() == 0)
      .onTrue(new InstantCommand(() -> stateManager.changeState(RobotState.ALGAE_REEF_INTAKE_L2)));
    new Trigger(() -> secondary.getPOV() == 90)
      .onTrue(new InstantCommand(() -> stateManager.changeState(RobotState.ALGAE_REEF_INTAKE_L3)));
    new Trigger(() -> secondary.getPOV() == 180)
      .onTrue(new InstantCommand(() -> stateManager.changeState(RobotState.ALGAE_OUTTAKE_PROCESSOR)));
  }

  /*
  public Command getAutonomousCommand() {
    return autoModeSelector.getAutonomousCommand();
  } */
}
