package frc.team2412.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.util.GeoConvertor;

public class FollowWpilibTrajectory extends CommandBase {
    public static class AutoConstants {
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 0.15;
        public static final double PY_CONTROLLER = 0.15;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints K_THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        public static final double MAX_SPEED_METERS_PER_SECOND = 1;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;
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
    SwerveControllerCommand swerveControllerCommand;
    Trajectory trajectory;

    public FollowWpilibTrajectory(DrivebaseSubsystem drivebaseSubsystem, Trajectory trajectory) {
        this.drivebaseSubsystem = drivebaseSubsystem;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        ProfiledPIDController thetaController = new ProfiledPIDController(
                0.1, 0, 0, AutoConstants.K_THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        trajectory.relativeTo(drivebaseSubsystem.getPoseAsPoseMeters());
        swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                drivebaseSubsystem::getPoseAsPoseMeters, // Functional interface to feed supplier
                AutoConstants.driveKinematics,

                // Position controllers
                new PIDController(AutoConstants.PX_CONTROLLER, 0, 0),
                new PIDController(AutoConstants.PY_CONTROLLER, 0, 0),
                thetaController,
                drivebaseSubsystem::updateModules,
                drivebaseSubsystem);
        swerveControllerCommand.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        swerveControllerCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return swerveControllerCommand.isFinished();
    }

    
}
