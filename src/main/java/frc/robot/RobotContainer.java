package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.RobotState;
import frc.robot.auto.AutoModeSelector;

public class RobotContainer {
  public SwerveSubsystem swerve;
  public VisionSubsystem vision;

  public PS4Controller primary;

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
    // this.coralManipulator = new CoralManipulator();

    this.primary = new PS4Controller(ControllerConstants.PRIMARY_PORT);

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

  private void configureBindings() {
    // PRIMARY CONTROLS
    new JoystickButton(primary, PS4Controller.Button.kCircle.value)
      .whileTrue(new RunCommand(() -> swerve.lockPose()));
    new JoystickButton(primary, PS4Controller.Button.kTriangle.value)
      .onTrue(new InstantCommand(() -> swerve.zeroGyro()));
  }

  public Command getAutonomousCommand() {
    return autoModeSelector.getAutonomousCommand();
  }
}