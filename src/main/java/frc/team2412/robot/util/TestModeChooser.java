package frc.team2412.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.climb.ClimbTestCommand;

public class TestModeChooser {

    private SendableChooser<TestMode> testModeChooser = new SendableChooser<>();
    private static Subsystems subsystems;

    public TestModeChooser(Subsystems subsystems) {
        this.subsystems = subsystems;

        testModeChooser.setDefaultOption(TestMode.CLIMB.uiName, TestMode.CLIMB);
        testModeChooser.addOption(TestMode.INDEX.uiName, TestMode.INDEX);
        testModeChooser.addOption(TestMode.INTAKE.uiName, TestMode.INTAKE);
        testModeChooser.addOption(TestMode.SHOOTER.uiName, TestMode.SHOOTER);

        ShuffleboardTab testTab = Shuffleboard.getTab("Tests");

        testTab.add("Choose Auto Mode", testModeChooser)
                .withPosition(0, 0)
                .withSize(2, 1);
    }

    public CommandBase getCommand() {
        return testModeChooser.getSelected().command;
    }

    public enum TestMode {
        // Replace with individual testing commands
        CLIMB(new ClimbTestCommand(subsystems.climbSubsystem), "Climb test"), INDEX(
                new ClimbTestCommand(subsystems.climbSubsystem),
                "Index test"), INTAKE(new ClimbTestCommand(subsystems.climbSubsystem),
                        "Intake test"), SHOOTER(new ClimbTestCommand(subsystems.climbSubsystem), "Shooter test");

        public final CommandBase command;
        public final String uiName;

        private TestMode(CommandBase command, String uiName) {
            this.command = command;
            this.uiName = uiName;
        }
    }
}
