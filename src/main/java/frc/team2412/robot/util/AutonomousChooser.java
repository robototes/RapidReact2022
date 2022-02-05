package frc.team2412.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.autonomous.Follow2910TrajectoryCommand;

public class AutonomousChooser {
    private AutonomousTrajectories trajectories;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("Square Path Auto", AutonomousMode.SQUARE_PATH_AUTO);
        autonomousModeChooser.addOption("Star Path Auto", AutonomousMode.STAR_PATH_AUTO);

        SmartDashboard.putData("Choose Auto Mode", autonomousModeChooser);
    }

    private SequentialCommandGroup getSquarePathAutoCommand(Subsystems subsystems) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
                new Follow2910TrajectoryCommand(subsystems.drivebaseSubsystem, trajectories.getSquarePathAuto()));

        return command;
    }

    private SequentialCommandGroup getStarPathAutoCommand(Subsystems subsystems) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(new Follow2910TrajectoryCommand(subsystems.drivebaseSubsystem, trajectories.getStarPathAuto()));

        return command;
    }

    public SequentialCommandGroup getCommand(Subsystems subsystems) {
        switch (autonomousModeChooser.getSelected()) {
            case SQUARE_PATH_AUTO:
                return getSquarePathAutoCommand(subsystems);
            case STAR_PATH_AUTO:
                return getStarPathAutoCommand(subsystems);
            default:
                return new SequentialCommandGroup();
        }
    }

    private enum AutonomousMode {
        SQUARE_PATH_AUTO, STAR_PATH_AUTO
    }
}
