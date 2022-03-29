package frc.team2412.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

public class WPILibFiveBallAutoCommand extends SequentialCommandGroup {
    public WPILibFiveBallAutoCommand(DrivebaseSubsystem drivebaseSubsystem) {

        TrajectoryConfig normalSpeedConfig = new TrajectoryConfig(1, 1)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(FollowWPILibTrajectory.WPILibAutoConstants.driveKinematics);
        ProfiledPIDController thetaController = new ProfiledPIDController(0.1, 0, 0, FollowWPILibTrajectory.WPILibAutoConstants.K_THETA_CONTROLLER_CONSTRAINTS);

        Trajectory trajectoryOne = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(8.4, 1.8, Rotation2d.fromDegrees(0)),
                        new Pose2d(7.4, 0.9, Rotation2d.fromDegrees(0)),
                        new Pose2d(5.3, 1.8, Rotation2d.fromDegrees(180))),
                normalSpeedConfig);
        Trajectory trajectoryTwo = TrajectoryGenerator.generateTrajectory(
                new Pose2d(7.4, 0.9, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(5.3, 1.8, Rotation2d.fromDegrees(180)), normalSpeedConfig);
        Trajectory trajectoryThree = TrajectoryGenerator.generateTrajectory(
                new Pose2d(5.3, 1.8, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(2.0, 1.3, Rotation2d.fromDegrees(0)), normalSpeedConfig);
        Trajectory trajectoryFour = TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.0, 1.3, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(5, 2.7, Rotation2d.fromDegrees(0)), normalSpeedConfig);

        addCommands(
            new FollowWPILibTrajectory(drivebaseSubsystem, trajectoryOne, thetaController),
            new FollowWPILibTrajectory(drivebaseSubsystem, trajectoryTwo, thetaController),
            new FollowWPILibTrajectory(drivebaseSubsystem, trajectoryThree, thetaController),
            new FollowWPILibTrajectory(drivebaseSubsystem, trajectoryFour, thetaController)
        );
    }
    
}
