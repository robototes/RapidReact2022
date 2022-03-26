package frc.team2412.robot.commands.autonomous;

import java.util.List;

import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

public class FiveBallAutoCommand extends SequentialCommandGroup {
    private final DrivebaseSubsystem drivebaseSubsystem;

    public FiveBallAutoCommand(DrivebaseSubsystem drivebaseSubsystem) {
        this.drivebaseSubsystem = drivebaseSubsystem;

        TrajectoryConfig fastConfig = new TrajectoryConfig(FollowWpilibTrajectory.AutoConstants.MAX_SPEED_METERS_PER_SECOND*2, FollowWpilibTrajectory.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED*2)
                .setKinematics(FollowWpilibTrajectory.AutoConstants.driveKinematics);

        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(8.335, 1.929), Rotation2d.fromDegrees(0)),
                List.of(), new Pose2d(new Translation2d(7.442, 0.763), Rotation2d.fromDegrees(0)),
                fastConfig);

        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(7.442, 0.763), Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(new Translation2d(5.176, 2.070), Rotation2d.fromDegrees(223)),
                fastConfig);

        Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(5.176, 2.070), Rotation2d.fromDegrees(223)),
                List.of(),
                new Pose2d(new Translation2d(1.624, 1.482), Rotation2d.fromDegrees(309)),
                fastConfig);

        Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(1.624, 1.482), Rotation2d.fromDegrees(309)),
                List.of(),
                new Pose2d(new Translation2d(4.271, 2.441), Rotation2d.fromDegrees(384)),
                fastConfig);

        addCommands(
                new FollowWpilibTrajectory(drivebaseSubsystem, trajectory1),
                new FollowWpilibTrajectory(drivebaseSubsystem, trajectory2),
                new FollowWpilibTrajectory(drivebaseSubsystem, trajectory3),
                new FollowWpilibTrajectory(drivebaseSubsystem, trajectory4)
        );
    }
}
