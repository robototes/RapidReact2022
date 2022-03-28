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

        TrajectoryConfig normalSpeedConfig = new TrajectoryConfig(FollowWpilibTrajectory.AutoConstants.MAX_SPEED_METERS_PER_SECOND*2, FollowWpilibTrajectory.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED*2)
                .setKinematics(FollowWpilibTrajectory.AutoConstants.driveKinematics);

        Trajectory trajectoryOne = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(8.4, 1.8, Rotation2d.fromDegrees(-90)),
                        new Pose2d(7.4, 0.9, Rotation2d.fromDegrees(180)),
                new Pose2d(5.3, 1.8, Rotation2d.fromDegrees(180))), normalSpeedConfig);
        Trajectory trajectoryTwo = TrajectoryGenerator.generateTrajectory(
                new Pose2d(7.4, 0.9, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(5.3, 1.8, Rotation2d.fromDegrees(180)), normalSpeedConfig);
        Trajectory trajectoryThree = TrajectoryGenerator.generateTrajectory(
                new Pose2d(5.3, 1.8, Rotation2d.fromDegrees(180)),
                List.of(),
                new Pose2d(2.0, 1.3, Rotation2d.fromDegrees(180)), normalSpeedConfig);
        Trajectory trajectoryFour = TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.0, 1.3, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(5, 2.7, Rotation2d.fromDegrees(0)), normalSpeedConfig);

        addCommands(
                new FollowWpilibTrajectory(drivebaseSubsystem, trajectoryOne),
                new FollowWpilibTrajectory(drivebaseSubsystem, trajectoryTwo),
                new FollowWpilibTrajectory(drivebaseSubsystem, trajectoryThree),
                new FollowWpilibTrajectory(drivebaseSubsystem, trajectoryFour)
        );
    }
}
