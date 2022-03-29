package frc.team2412.robot.commands.autonomous;


import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.commands.index.ShootCommand;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.TargetLocalizer;

public class JackFiveBallAutoCommand extends SequentialCommandGroup {
    private final DrivebaseSubsystem drivebaseSubsystem;

    public JackFiveBallAutoCommand(DrivebaseSubsystem drivebaseSubsystem, IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem, TargetLocalizer localizer) {
        this.drivebaseSubsystem = drivebaseSubsystem;

        // go down to first ball, arc down to pick it up
        // go to second ball, stop and shoot imediately
        // move to third ball, stop and intake for collection
        // move to where second ball was and shoot

        Trajectory trajectory1 = new Trajectory(
                new SimplePathBuilder(new Vector2(328, 75.551), Rotation2.fromDegrees(-90))
                .arcTo(new Vector2(293.811, 29.771), new Vector2(281.299, 92.401), Rotation2.fromDegrees(-90))
                .lineTo(new Vector2(202.029, 75.188), Rotation2.fromDegrees(125))
                .build(),
                DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1
        );

        Trajectory trajectory2 = new Trajectory(
                new SimplePathBuilder(new Vector2(202.029, 75.188), Rotation2.fromDegrees(125))
                .lineTo(new Vector2(66.456, 58.818), Rotation2.fromDegrees(202))
                .build(),
                DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1
        );

        Trajectory trajectory3 = new Trajectory(
                new SimplePathBuilder(new Vector2(66.456, 58.818), Rotation2.fromDegrees(202))
                .lineTo(new Vector2(207.029, 82.188), Rotation2.fromDegrees(125))
                .build(),
                DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS, 0.1
        );

        addCommands(
                new Follow2910TrajectoryCommand(drivebaseSubsystem, trajectory1),
                new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new ShootCommand(indexSubsystem, shooterSubsystem, localizer)
                ),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, trajectory2),
                new WaitCommand(2),
                new Follow2910TrajectoryCommand(drivebaseSubsystem, trajectory3),
                new ParallelDeadlineGroup(
                        new WaitCommand(3),
                        new ShootCommand(indexSubsystem, shooterSubsystem, localizer)
                )
        );
    }
}
