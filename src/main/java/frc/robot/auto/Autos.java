package frc.robot.auto;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public final class Autos {
  private final RobotContainer c;
  private final Map<String, Supplier<Command>> reg = new LinkedHashMap<>();
  private final SendableChooser<String> chooser = new SendableChooser<>();

  public Autos(RobotContainer c) {
    this.c = c;
    add("Do Nothing", () -> Commands.none(), true);
  }

  public void add(String name, Supplier<Command> f) { add(name, f, false); }
  public void add(String name, Function<RobotContainer, Command> f) { add(name, () -> f.apply(c), false); }
  public void add(AutoCommand ac) { add(ac.name(), () -> ac.get(), false); }

  private void add(String name, Supplier<Command> f, boolean deflt) {
    reg.put(name, f);
    if (deflt) chooser.setDefaultOption(name, name);
    else chooser.addOption(name, name);
  }

  public void publish() { SmartDashboard.putData("Auto", chooser); }

  public Command selected() {
    String key = chooser.getSelected();
    if (key == null) return Commands.none();
    return reg.getOrDefault(key, () -> Commands.none()).get();
  }
}
