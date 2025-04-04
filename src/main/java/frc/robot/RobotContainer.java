package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MotorTest;
import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.AlgaeManipConstants.AlgaeManipState;
import frc.robot.Constants.CoralManipConstants.CoralManipAngleState;
import frc.robot.Constants.CoralManipConstants.CoralManipState;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Constants.MotorTestConstants.MotorTestState;

public class RobotContainer {
  public SwerveSubsystem swerve;
  // public VisionSubsystem vision;
  // public AlgaeManipulator algaeManipulator;
  // public CoralManipulator coralManipulator;
  // public Elevator elevator;

  public PS4Controller primary;

  public RobotContainer() {
    try { this.swerve = new SwerveSubsystem(); }
    catch (Exception e) {
      System.err.println("YAGSL failed to read our drivetrain swerve configuration! D:");
      e.printStackTrace();
      System.exit(1);
    }
    // this.vision = new VisionSubsystem(swerve);
    // this.algaeManipulator = new AlgaeManipulator();
    // this.coralManipulator = new CoralManipulator();
    // this.elevator = new Elevator();

    this.primary = new PS4Controller(ControllerConstants.PRIMARY_PORT); 
    // this.secondary = new PS4Controller(ControllerConstants.SECONDARY_PORT);
    // this.primary = new Joystick(ControllerConstants.PRIMARY_PORT);
    
    configureBindings();

    swerve.setDefaultCommand(
      new RunCommand(
        () -> {
          // get speeds & apply deadbands
          double xSpeed = -MathUtil.applyDeadband(primary.getLeftY(), ControllerConstants.DEADBAND);
          double ySpeed = -MathUtil.applyDeadband(primary.getLeftX(), ControllerConstants.DEADBAND);
          double rotSpeed = -MathUtil.applyDeadband(primary.getRightX(), ControllerConstants.DEADBAND);
          /*
          double xSpeed = -MathUtil.applyDeadband(primary.getRawAxis(1), ControllerConstants.DEADBAND);
          double ySpeed = -MathUtil.applyDeadband(primary.getRawAxis(0), ControllerConstants.DEADBAND);
          double rotSpeed = -MathUtil.applyDeadband(primary.getRawAxis(4), ControllerConstants.DEADBAND);
          */

          // square speeds for better control
          xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
          ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
          rotSpeed = Math.copySign(rotSpeed * rotSpeed, rotSpeed);

          // apply multipliers
          xSpeed *= swerve.getMaxSpeed();
          ySpeed *= swerve.getMaxSpeed();
          rotSpeed *= ControllerConstants.ROTATION_SPEED;

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
    /* 
    new JoystickButton(primary, 1) // A
      .whileTrue(new InstantCommand(() -> motorTest.setState(MotorTestState.FORWARD)))
      .onFalse(new InstantCommand(() -> motorTest.setState(MotorTestState.REST)));
    
    new JoystickButton(primary, 2) // B
      .whileTrue(new InstantCommand(() -> motorTest.setState(MotorTestState.REVERSE)))
      .onFalse(new InstantCommand(() -> motorTest.setState(MotorTestState.REST)));
    */
    // primary controls
    /*
    new JoystickButton(primary, PS4Controller.Button.kTriangle.value)
      .onTrue(new InstantCommand(swerve::zeroGyro));
    new JoystickButton(primary, PS4Controller.Button.kCircle.value)
      .onTrue(new InstantCommand(swerve::lockPose));
    */

    // elevator
    /*
    new JoystickButton(secondary, PS4Controller.Button.kCross.value)
      .onTrue(new InstantCommand(() -> elevator.setState(ElevatorState.DOWN)));
    new JoystickButton(secondary, PS4Controller.Button.kTriangle.value)
      .onTrue(new InstantCommand(() -> elevator.setState(ElevatorState.MID)));
    new JoystickButton(secondary, PS4Controller.Button.kCircle.value)
      .onTrue(new InstantCommand(() -> elevator.setState(ElevatorState.MAX)));

    // algae
    new JoystickButton(secondary, PS4Controller.Button.kL1.value)
      .whileTrue(new RunCommand(() -> algaeManipulator.setState(AlgaeManipState.INTAKE)))
      .onFalse(new InstantCommand(() -> algaeManipulator.setState(AlgaeManipState.IDLE)));
    new JoystickButton(secondary, PS4Controller.Button.kR1.value)
      .whileTrue(new RunCommand(() -> algaeManipulator.setState(AlgaeManipState.RELEASE)))
      .onFalse(new InstantCommand(() -> algaeManipulator.setState(AlgaeManipState.IDLE)));
    new JoystickButton(secondary, PS4Controller.Button.kSquare.value)
      .onTrue(new InstantCommand(algaeManipulator::togglePivotState));

    // coral
    new JoystickButton(secondary, PS4Controller.Button.kL2.value)
      .whileTrue(new InstantCommand(() -> {
        coralManipulator.setState(CoralManipState.INTAKE);
        coralManipulator.setState(CoralManipAngleState.INTAKE);
      })).onFalse(new InstantCommand(() -> {
        coralManipulator.setState(CoralManipState.REST);
        coralManipulator.setState(CoralManipAngleState.IDLE);
      }));
    new JoystickButton(secondary, PS4Controller.Button.kR2.value)
      .whileTrue(new InstantCommand(() -> 
        coralManipulator.setState(CoralManipAngleState.SCORING)
      )).onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> coralManipulator.setState(CoralManipState.OUTTAKE)),
        new WaitCommand(0.5),
        new InstantCommand(() -> {
            coralManipulator.setState(CoralManipState.REST);
            coralManipulator.setState(CoralManipAngleState.IDLE);
        })
    ));
    */
  }

  // public Command getAutonomousCommand() {}
}
