package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.subsystems.AlgaeManipulator;
// import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.ControllerConstants;

public class RobotContainer {
  public SwerveSubsystem swerve;
  public VisionSubsystem vision;
  // public AlgaeManipulator algaeManipulator;
  // public CoralManipulator coralManipulator;
  public PS4Controller controller = new PS4Controller(ControllerConstants.CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
    try { this.swerve = new SwerveSubsystem(); }
    catch (Exception e) {
      System.err.println("YAGSL failed to read our drivetrain swerve configuration! D:");
      e.printStackTrace();
      System.exit(1);
    }
    this.vision = new VisionSubsystem(swerve);
    // this.algaeManipulator = new AlgaeManipulator();
    // this.coralManipulator = new CoralManipulator();

    swerve.setDefaultCommand(
      new RunCommand(
        () -> {
          // get speeds & apply deadbands
          double xSpeed = -MathUtil.applyDeadband(controller.getLeftY(), ControllerConstants.DEADBAND);
          double ySpeed = -MathUtil.applyDeadband(controller.getLeftX(), ControllerConstants.DEADBAND);
          double rotSpeed = -MathUtil.applyDeadband(controller.getRightX(), ControllerConstants.DEADBAND);

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
    new JoystickButton(controller, PS4Controller.Button.kTriangle.value)
      .onTrue(new InstantCommand(swerve::zeroGyro));
    new JoystickButton(controller, PS4Controller.Button.kCircle.value)
      .onTrue(new InstantCommand(swerve::lockPose));
  }

  // public Command getAutonomousCommand() {}
}
