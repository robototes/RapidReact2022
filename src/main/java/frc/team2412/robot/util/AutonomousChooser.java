package frc.team2412.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.autonomous.AutonomousCommand;
import frc.team2412.robot.commands.autonomous.Follow2910TrajectoryCommand;

public class AutonomousChooser {
    private AutonomousTrajectories trajectories;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("Square Path Auto", AutonomousMode.SQUARE_PATH_AUTO);
        autonomousModeChooser.addOption("Star Path Auto", AutonomousMode.STAR_PATH_AUTO);
        autonomousModeChooser.addOption("Wpi Lib Auto", AutonomousMode.WPI_PATH_AUTO);

        ShuffleboardTab drivebaseTab = Shuffleboard.getTab("Drivebase");
        drivebaseTab.add("Choose Auto Mode", autonomousModeChooser)
                .withPosition(0, 3)
                .withSize(2, 1);
    }

    private SequentialCommandGroup getSquarePathAutoCommand(Subsystems subsystems) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
                new Follow2910TrajectoryCommand(subsystems.drivebaseSubsystem, trajectories.getSquarePathAuto()));

        return command;
    }

    private SequentialCommandGroup getStarPathAutoCommand(Subsystems subsystems) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
                new Follow2910TrajectoryCommand(subsystems.drivebaseSubsystem, trajectories.getStarPathAuto()));

        return command;
    }
    private SequentialCommandGroup getAutoWPICommand (Subsystems subsystems){
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(new AutonomousCommand(subsystems.drivebaseSubsystem).getAutonomousCommand());
        return command;
    }

    public SequentialCommandGroup getCommand(Subsystems subsystems) {
        switch (autonomousModeChooser.getSelected()) {
            case SQUARE_PATH_AUTO:
                System.out.println("Square path auto chosen");
                return getSquarePathAutoCommand(subsystems);
            case STAR_PATH_AUTO:
                System.out.println("Star path auto chosen");
                return getStarPathAutoCommand(subsystems);
            case WPI_PATH_AUTO:
                System.out.println("Wpi lib path chosen");
                return getAutoWPICommand(subsystems);
            default:
                System.out.println("No auto path chosen");
                return new SequentialCommandGroup();
        }
    }

    private enum AutonomousMode {
        SQUARE_PATH_AUTO, STAR_PATH_AUTO, WPI_PATH_AUTO
    }
}
