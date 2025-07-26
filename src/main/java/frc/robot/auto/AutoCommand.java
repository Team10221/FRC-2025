package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public abstract class AutoCommand {
  protected final RobotContainer c;
  private final String name;

  public AutoCommand(String name, RobotContainer c) {
    this.name = name;
    this.c = c;
  }

  public abstract Command get();

  @Override public String toString() { return name; }
  public String name() { return name; }
}
