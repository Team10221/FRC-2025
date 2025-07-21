package frc.robot.auto;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AutoModeSelector {
    /*
    public enum DesiredMode {
        TEST_AUTO,
        DO_NOTHING,
        LEAVE
    }

    private final SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();
    private final RobotContainer container;

    public AutoModeSelector(RobotContainer container) {
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Simple Path", DesiredMode.TEST_AUTO);
        mModeChooser.addOption("Leave Auto", DesiredMode.LEAVE);

        SmartDashboard.putData("Auto Mode", mModeChooser);

        this.container = container;
    }

    public SendableChooser<DesiredMode> getModeChooser() {
        return mModeChooser;
    }

    public Command getAutonomousCommand() {
        if (mModeChooser.getSelected() == null) {
            return Commands.none();
        }
        switch (mModeChooser.getSelected()) {
            case DO_NOTHING:
                return Commands.none();
            case TEST_AUTO:
                return AutoFactory.followChoreoTrajectory("Score_H", container);
            case LEAVE:
                return AutoFactory.leaveCommand(container);
                //return AutoFactory.followChoreoTrajectory("Straight_Line", container);
            default:
                return Commands.none();
        }
    } */
}