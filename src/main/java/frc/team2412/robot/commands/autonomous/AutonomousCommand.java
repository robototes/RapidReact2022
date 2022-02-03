package frc.team2412.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.util.Constants;
import frc.team2412.robot.util.GeoConvertor;

import java.util.List;

public class AutonomousCommand extends SequentialCommandGroup {
    DrivebaseSubsystem drivebaseSubsystem;

    public AutonomousCommand(DrivebaseSubsystem d){
        drivebaseSubsystem = d;
    }
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.maxSpeedMetersPerSecond,
                        Constants.AutoConstants.maxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.driveKinematics);

        // An example trajectory to follow.  All units in meters.
//        Trajectory exampleTrajectory =
//                TrajectoryGenerator.generateTrajectory(
//                        // Start at the origin facing the +X direction
//                        new Pose2d(0, 0, new Rotation2d(0)),
//                        // Pass through these two interior waypoints, making an 's' curve path
//                        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//                        // End 3 meters straight ahead of where we started, facing forward
//                        new Pose2d(3, 0, new Rotation2d(0)),
//                        config);

        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(-0.01, 0)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(-0.3, 0, new Rotation2d(180)),
                        config);



//        ProfiledPIDController thetaController =
//                new ProfiledPIDController(
//                        Constants.AutoConstants.PThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
//        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                       0.000000005, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
                new SwerveControllerCommand(
                        exampleTrajectory,
                        drivebaseSubsystem::getPoseAsPose, // Functional interface to feed supplier
                        Constants.DriveConstants.driveKinematics,

                        // Position controllers
                        new PIDController(Constants.AutoConstants.PXController, 0, 0),
                        new PIDController(Constants.AutoConstants.PYController, 0, 0),
                        thetaController,
                        drivebaseSubsystem::updateModules,
                        drivebaseSubsystem);


        System.out.println(exampleTrajectory.getTotalTimeSeconds());

        // Reset odometry to the starting pose of the trajectory.
        drivebaseSubsystem.resetPose(exampleTrajectory.getInitialPose());

        System.out.println("Created Trajectory");
        // Run path following command, then stop at the end.
        //have to fix this later, the parameters are just placeholders to get program to build
        return swerveControllerCommand.andThen(() -> drivebaseSubsystem.drive(GeoConvertor.translation2dToVector2(exampleTrajectory.getInitialPose().getTranslation()),
                0, false));
    }

}

