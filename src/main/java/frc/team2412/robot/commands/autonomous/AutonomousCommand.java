package frc.team2412.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {
    public static class AutoConstants {
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 0.15;
        public static final double PY_CONTROLLER = 0.15;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints K_THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        public static final double MAX_SPEED_METERS_PER_SECOND = 0.3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.1;
        public static final double TRACK_WIDTH = 1.0;
        // Distance between centers of right and left wheels on robot
        public static final double WHEEL_BASE = 1.0;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));
    }

    DrivebaseSubsystem drivebaseSubsystem;

    /**
     * Subsystems: {@link DrivebaseSubsystem}
     */
    public AutonomousCommand() {
        drivebaseSubsystem = DrivebaseSubsystem.instance;
    }

    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(AutoConstants.driveKinematics);
        // creating trajectory path (right now is a square)
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(new Translation2d(1, 0), new Translation2d(1, 1), new Translation2d(0, 1)),
                new Pose2d(0, 0.0, Rotation2d.fromDegrees(0)), config);
        // creates thetacontroller (rotation)
        ProfiledPIDController thetaController = new ProfiledPIDController(0.000000005, 0, 0,
                AutoConstants.K_THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        exampleTrajectory.relativeTo(drivebaseSubsystem.getPoseAsPoseMeters());
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
                drivebaseSubsystem::getPoseAsPoseMeters, // Functional interface
                                                            // to feed supplier
                AutoConstants.driveKinematics,

                // Position controllers
                new PIDController(AutoConstants.PX_CONTROLLER, 0, 0),
                new PIDController(AutoConstants.PY_CONTROLLER, 0, 0), thetaController,
                drivebaseSubsystem::updateModules, drivebaseSubsystem);

        return swerveControllerCommand;
    }
}
