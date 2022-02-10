package frc.team2412.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.util.Constants;

import java.util.List;

public class AutonomousCommand extends SequentialCommandGroup {
    public static class AutoConstants{
        public static final double MaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double MaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double PXController = 0.15;
        public static final double PYController = 0.15;
        public static final double PThetaController = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                MaxAngularSpeedRadiansPerSecond, MaxAngularSpeedRadiansPerSecondSquared);

        public static final double maxSpeedMetersPerSecond = 0.3;
        public static final double maxAccelerationMetersPerSecondSquared = 0.1;
    }
    DrivebaseSubsystem drivebaseSubsystem;

    public AutonomousCommand(DrivebaseSubsystem d) {
        drivebaseSubsystem = d;
    }

    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.maxSpeedMetersPerSecond,
                AutoConstants.maxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.driveKinematics);
        // creating trajectory path (right now is a square)
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(new Translation2d(1, 0), new Translation2d(1, 1), new Translation2d(0, 1)),
                new Pose2d(0, 0.0, Rotation2d.fromDegrees(0)),
                config);
        // creates thetacontroller (rotation)
        ProfiledPIDController thetaController = new ProfiledPIDController(
                0.000000005, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                drivebaseSubsystem::getPoseAsPoseMeters, // Functional interface to feed supplier
                Constants.DriveConstants.driveKinematics,

                // Position controllers
                new PIDController(AutoConstants.PXController, 0, 0),
                new PIDController(AutoConstants.PYController, 0, 0),
                thetaController,
                drivebaseSubsystem::updateModules,
                drivebaseSubsystem);

        System.out.println(exampleTrajectory.getTotalTimeSeconds());

        // Reset odometry to the starting pose of the trajectory.
        drivebaseSubsystem.resetPose(exampleTrajectory.getInitialPose());

        System.out.println("Created Trajectory");
        // Run path following command, then stop at the end.
        // have to fix this later, the parameters are just placeholders to get program
        // to build
        // first paratmer used to be that we ocmented out
        // GeoConvertor.translation2dToVector2(exampleTrajectory.getInitialPose().getTranslation())
        return swerveControllerCommand;
    }

}
