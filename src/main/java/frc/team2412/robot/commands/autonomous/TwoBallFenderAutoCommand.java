package frc.team2412.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

public class TwoBallFenderAutoCommand extends SequentialCommandGroup {
    private final DrivebaseSubsystem drivebaseSubsystem;

    public TwoBallFenderAutoCommand(DrivebaseSubsystem drivebaseSubsystem) {
        this.drivebaseSubsystem = drivebaseSubsystem;

        TrajectoryConfig fastConfig = new TrajectoryConfig(1, 0.8)
                .setKinematics(FollowWPILibTrajectory.WPILibAutoConstants.driveKinematics);
        ProfiledPIDController thetaController = new ProfiledPIDController(0.1, 0, 0,
                FollowWPILibTrajectory.WPILibAutoConstants.K_THETA_CONTROLLER_CONSTRAINTS);

        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(5.89, 5.1), Rotation2d.fromDegrees(46)),
                List.of(),
                new Pose2d(new Translation2d(4.99, 5.97), Rotation2d.fromDegrees(46)),
                fastConfig);

        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(4.99, 5.97), Rotation2d.fromDegrees(46)),
                List.of(),
                new Pose2d(new Translation2d(6.98, 4.48), Rotation2d.fromDegrees(-21)),
                fastConfig);

        addCommands(
                new FollowWPILibTrajectory(drivebaseSubsystem, trajectory1, thetaController),
                new FollowWPILibTrajectory(drivebaseSubsystem, trajectory2, thetaController));
    }
}
