package frc.team2412.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.autonomous.AutonomousCommand;
import frc.team2412.robot.commands.autonomous.Follow2910TrajectoryCommand;
import frc.team2412.robot.commands.climb.ClimbTestCommand;

public class AutonomousChooser {

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();
    private static Subsystems subsystems;
    private static AutonomousTrajectories trajectories;

    public AutonomousChooser(Subsystems subsystems, AutonomousTrajectories trajectories) {
        this.subsystems = subsystems;
        AutonomousChooser.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption(AutonomousMode.SQUARE_PATH.uiName, AutonomousMode.SQUARE_PATH);

        for (var mode : AutonomousMode.values()) {
            if (mode != AutonomousMode.SQUARE_PATH) {
                autonomousModeChooser.addOption(mode.uiName, mode);
            }
        }

        ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");

        autonomousTab.add("Choose Auto Mode", autonomousModeChooser)
                .withPosition(0, 0)
                .withSize(2, 1);
    }

    public CommandBase getCommand() {
        return autonomousModeChooser.getSelected().command;
    }

    private static SequentialCommandGroup getSquarePathAutoCommand(Subsystems subsystems) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
                new Follow2910TrajectoryCommand(subsystems.drivebaseSubsystem, trajectories.getSquarePathAuto()));

        return command;
    }

    private static SequentialCommandGroup getStarPathAutoCommand(Subsystems subsystems) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
                new Follow2910TrajectoryCommand(subsystems.drivebaseSubsystem, trajectories.getStarPathAuto()));

        return command;
    }

    private static SequentialCommandGroup getAutoWPICommand(Subsystems subsystems) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(new AutonomousCommand(subsystems.drivebaseSubsystem).getAutonomousCommand());
        return command;
    }

    public enum AutonomousMode {
        // Replace with individual testing commands
        // spotless:off
        STAR_PATH(AutonomousChooser.getStarPathAutoCommand(subsystems), "Star Test"),
        SQUARE_PATH(AutonomousChooser.getSquarePathAutoCommand(subsystems), "Square Path"),
        WPI_PATH(AutonomousChooser.getAutoWPICommand(subsystems), "WPI Lib Path"),
        CLIMB(new ClimbTestCommand(subsystems.climbSubsystem), "Climb test"), 
        INDEX(new ClimbTestCommand(subsystems.climbSubsystem), "Index test"), 
        INTAKE(new ClimbTestCommand(subsystems.climbSubsystem), "Intake test"), 
        SHOOTER(new ClimbTestCommand(subsystems.climbSubsystem), "Shooter test");
        // spotless:on

        public final CommandBase command;
        public final String uiName;

        private AutonomousMode(CommandBase command, String uiName) {
            this.command = command;
            this.uiName = uiName;
        }
    }
}
