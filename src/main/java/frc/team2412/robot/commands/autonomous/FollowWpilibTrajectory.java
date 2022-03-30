package frc.team2412.robot.commands.autonomous;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class FollowWpilibTrajectory extends CommandBase {
    public static class WPILibAutoConstants {
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 0.3;
        public static final double PY_CONTROLLER = 0.3;

        public static final double DEFAULT_THETA = 0.1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints K_THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

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

    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private final Supplier<Pose2d> pose;
    private final HolonomicDriveController controller;
    private final Consumer<ChassisSpeeds> outputModuleStates;
    private final Supplier<Rotation2d> desiredRotation;

    /**
     * Constructs a new SwerveControllerCommand that when executed will follow the
     * provided
     * trajectory. This command will not return output voltages but rather raw
     * module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * @param drivebase
     * @param trajectory
     *            The trajectory to follow.
     * @param pose
     *            A function that supplies the robot pose - use one of the odometry
     *            classes to
     *            provide this.
     * @param thetaController
     *            The Trajectory Tracker PID controller for angle for the robot.
     */
    @SuppressWarnings("ParameterName")
    public FollowWpilibTrajectory(
            DrivebaseSubsystem drivebase,
            Trajectory trajectory,
            ProfiledPIDController thetaController,
            Supplier<Rotation2d> desiredRotation) {
        this.pose = drivebase::getPoseAsPoseMeters;
        this.trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.trajectory.relativeTo(pose.get());

        controller = new HolonomicDriveController(
                new PIDController(WPILibAutoConstants.PX_CONTROLLER, 0, 0),
                new PIDController(WPILibAutoConstants.PY_CONTROLLER, 0, 0),
                requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand"));

        this.outputModuleStates = drivebase::updateModules;

        this.desiredRotation = requireNonNullParam(desiredRotation, "desiredRotation", "SwerveControllerCommand");

        addRequirements(drivebase);
    }

    /**
     * Constructs a new SwerveControllerCommand that when executed will follow the
     * provided
     * trajectory. This command will not return output voltages but rather raw
     * module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * <p>
     * Note 2: The final rotation of the robot will be set to the rotation of the
     * final pose in the
     * trajectory. The robot will not follow the rotations from the poses at each
     * timestep. If
     * alternate rotation behavior is desired, the other constructor with a supplier
     * for rotation
     * should be used.
     *
     * @param trajectory
     *            The trajectory to follow.
     * @param pose
     *            A function that supplies the robot pose - use one of the odometry
     *            classes to
     *            provide this.
     * @param thetaController
     *            The Trajectory Tracker PID controller for angle for the robot.
     */
    @SuppressWarnings("ParameterName")
    public FollowWpilibTrajectory(
            DrivebaseSubsystem drivebase,
            Trajectory trajectory,
            ProfiledPIDController thetaController) {
        this(
                drivebase,
                trajectory,
                thetaController,
                () -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation());
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        // gets current time and desired location in trajectory
        double curTime = timer.get();
        var desiredState = trajectory.sample(curTime);

        var targetChassisSpeeds = controller.calculate(pose.get(), desiredState, desiredRotation.get());
        outputModuleStates.accept(targetChassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        outputModuleStates.accept(new ChassisSpeeds(0, 0, 0));

        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
