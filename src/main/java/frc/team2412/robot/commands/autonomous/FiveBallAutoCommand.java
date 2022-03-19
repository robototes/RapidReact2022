package frc.team2412.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

public class FiveBallAutoCommand extends SequentialCommandGroup {
    private final DrivebaseSubsystem drivebaseSubsystem;

    public FiveBallAutoCommand(DrivebaseSubsystem drivebaseSubsystem) {
        this.drivebaseSubsystem = drivebaseSubsystem;

        TrajectoryConfig fastConfig = new TrajectoryConfig(1, 0.8)
                .setKinematics(FollowWpilibTrajectory.AutoConstants.driveKinematics);
        TrajectoryConfig slowConfig = new TrajectoryConfig(0.2, 1)
                .setKinematics(FollowWpilibTrajectory.AutoConstants.driveKinematics);

        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(7.5, 1.9), Rotation2d.fromDegrees(0)),
                List.of(), new Pose2d(new Translation2d(7.3, 1.1), Rotation2d.fromDegrees(0)),
                fastConfig
        );

        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(7.3, 1.1), Rotation2d.fromDegrees(0)),
                List.of(new Translation2d(5.1, 1.8)),
                new Pose2d(new Translation2d(2.1, 1.3), Rotation2d.fromDegrees(0)),
                fastConfig
        );

        Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(2.1, 1.3), Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(new Translation2d(5, 2.7), Rotation2d.fromDegrees(180)),
                fastConfig
        );

        Trajectory mainTrajectory = trajectory3.concatenate(trajectory2.concatenate(trajectory1));

        addCommands(
                new FollowWpilibTrajectory(drivebaseSubsystem, mainTrajectory)
        );
    }
}
