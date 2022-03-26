package frc.team2412.robot.commands.autonomous;

import java.util.List;

import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

public class JackFiveBallAutoCommand extends SequentialCommandGroup {
    private final DrivebaseSubsystem drivebaseSubsystem;

    public JackFiveBallAutoCommand(DrivebaseSubsystem drivebaseSubsystem) {
        this.drivebaseSubsystem = drivebaseSubsystem;

        Trajectory trajectory1 = new Trajectory(
                new SimplePathBuilder(new Vector2(338.4, 72.531), Rotation2.fromDegrees(0))
                .lineTo(new Vector2(300.765, 13.201))
                .lineTo(new Vector2(202.029, 75.188), Rotation2.fromDegrees(90))
                .lineTo(new Vector2(44.406, 46.851))
                .lineTo(new Vector2(191.403, 103.967), Rotation2.fromDegrees(0))
                .build(),
                DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1
        );

        addCommands(
                new Follow2910TrajectoryCommand(drivebaseSubsystem, trajectory1)
        );
    }
}
