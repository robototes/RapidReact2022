package frc.team2412.robot.commands.autonomous;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

import java.util.List;

public class AutonomousCommand extends SequentialCommandGroup {
    public static class AutoConstants {
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 0.3;
        public static final double PY_CONTROLLER = 0.3;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints K_THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        public static final double MAX_SPEED_METERS_PER_SECOND = 2;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;
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

    public AutonomousCommand(DrivebaseSubsystem d) {
        drivebaseSubsystem = d;

        // Create normalSpeedConfig for trajectory
        TrajectoryConfig normalSpeedConfig = new TrajectoryConfig(
                AutonomousCommand.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutonomousCommand.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(AutonomousCommand.AutoConstants.driveKinematics);
        TrajectoryConfig fastSpeedConfig = new TrajectoryConfig(
                AutonomousCommand.AutoConstants.MAX_SPEED_METERS_PER_SECOND * 2,
                AutonomousCommand.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED * 2)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(AutonomousCommand.AutoConstants.driveKinematics);
        Trajectory trajectoryOne = TrajectoryGenerator.generateTrajectory(
                new Pose2d(7.5, 1.9, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(7.3, 1.1, Rotation2d.fromDegrees(-90)),
                fastSpeedConfig);
        Trajectory trajectoryTwo = TrajectoryGenerator.generateTrajectory(
                new Pose2d(7.3, 1.1, Rotation2d.fromDegrees(180)),
                List.of(),
                new Pose2d(5.1, 1.8, Rotation2d.fromDegrees(180)), normalSpeedConfig);
        Trajectory trajectoryThree = TrajectoryGenerator.generateTrajectory(
                new Pose2d(5.1, 1.8, Rotation2d.fromDegrees(180)),
                List.of(),
                new Pose2d(2.1, 1.3, Rotation2d.fromDegrees(180)), normalSpeedConfig);
        Trajectory trajectoryFour = TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.1, 1.3, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(5, 2.7, Rotation2d.fromDegrees(0)), normalSpeedConfig);
        ;

        ProfiledPIDController thetaController = new ProfiledPIDController(
                4, 0, 0, AutoConstants.K_THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        trajectoryOne.relativeTo(drivebaseSubsystem.getPoseAsPoseMeters());
        trajectoryTwo.relativeTo(drivebaseSubsystem.getPoseAsPoseMeters());
        trajectoryThree.relativeTo(drivebaseSubsystem.getPoseAsPoseMeters());
        trajectoryFour.relativeTo(drivebaseSubsystem.getPoseAsPoseMeters());

        SwerveControllerCommand swerveControllerCommandOne = new SwerveControllerCommand(
                trajectoryOne,
                drivebaseSubsystem::getPoseAsPoseMeters, // Functional interface to feed supplier
                AutoConstants.driveKinematics,

                // Position controllers
                new PIDController(AutoConstants.PX_CONTROLLER, 0, 0),
                new PIDController(AutoConstants.PY_CONTROLLER, 0, 0),
                thetaController,
                drivebaseSubsystem::updateModules,
                drivebaseSubsystem);
        SwerveControllerCommand swerveControllerCommandTwo = new SwerveControllerCommand(
                trajectoryTwo,
                drivebaseSubsystem::getPoseAsPoseMeters, // Functional interface to feed supplier
                AutoConstants.driveKinematics,

                // Position controllers
                new PIDController(AutoConstants.PX_CONTROLLER, 0, 0),
                new PIDController(AutoConstants.PY_CONTROLLER, 0, 0),
                thetaController,
                drivebaseSubsystem::updateModules,
                drivebaseSubsystem);
        SwerveControllerCommand swerveControllerCommandThree = new SwerveControllerCommand(
                trajectoryThree,
                drivebaseSubsystem::getPoseAsPoseMeters, // Functional interface to feed supplier
                AutoConstants.driveKinematics,

                // Position controllers
                new PIDController(AutoConstants.PX_CONTROLLER, 0, 0),
                new PIDController(AutoConstants.PY_CONTROLLER, 0, 0),
                thetaController,
                drivebaseSubsystem::updateModules,
                drivebaseSubsystem);
        SwerveControllerCommand swerveControllerCommandFour = new SwerveControllerCommand(
                trajectoryFour,
                drivebaseSubsystem::getPoseAsPoseMeters, // Functional interface to feed supplier
                AutoConstants.driveKinematics,

                // Position controllers
                new PIDController(AutoConstants.PX_CONTROLLER, 0, 0),
                new PIDController(AutoConstants.PY_CONTROLLER, 0, 0),
                thetaController,
                drivebaseSubsystem::updateModules,
                drivebaseSubsystem);

        addCommands(
                // new ParallelCommandGroup(
                // new ScheduleCommand(new ShooterTargetCommand(shooterSubsystem, localizer)),
                // new WaitCommand(1)),
                // new ParallelDeadlineGroup(new WaitCommand(1), new IndexShootCommand(indexSubsystem)),
                // new IntakeSetExtendCommand(intakeSubsystem),
                swerveControllerCommandOne, swerveControllerCommandTwo, swerveControllerCommandThree,
                swerveControllerCommandFour,
                new SequentialCommandGroup());

    }

    // public Command getAutonomousCommand(Trajectory exampleTrajectory) {
    //
    //
    // }
}
