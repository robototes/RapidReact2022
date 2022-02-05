package frc.team2412.robot.commands.autonomous;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class SwerveControllerCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private final Supplier<Pose2d> pose;
    private final SwerveDriveKinematics kinematics;
    private final HolonomicDriveController controller;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    private final Supplier<Rotation2d> desiredRotation;
    private final DrivebaseSubsystem drivebase;

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
     * @param trajectory
     *            The trajectory to follow.
     * @param pose
     *            A function that supplies the robot pose - use one of the odometry
     *            classes to
     *            provide this.
     * @param kinematics
     *            The kinematics for the robot drivetrain.
     * @param xController
     *            The Trajectory Tracker PID controller for the robot's x position.
     * @param yController
     *            The Trajectory Tracker PID controller for the robot's y position.
     * @param thetaController
     *            The Trajectory Tracker PID controller for angle for the robot.
     * @param desiredRotation
     *            The angle that the drivetrain should be facing. This is sampled at
     *            each
     *            time step.
     * @param outputModuleStates
     *            The raw output module states from the position controllers.
     * @param drivebase
     * @param requirements
     *            The subsystems to require.
     */
    @SuppressWarnings("ParameterName")
    public SwerveControllerCommand(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Supplier<Rotation2d> desiredRotation,
            Consumer<SwerveModuleState[]> outputModuleStates,
            DrivebaseSubsystem drivebase, Subsystem... requirements) {
        this.trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
        this.pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");
        this.kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveControllerCommand");
        this.drivebase = drivebase;

        controller = new HolonomicDriveController(
                requireNonNullParam(xController, "xController", "SwerveControllerCommand"),
                requireNonNullParam(yController, "xController", "SwerveControllerCommand"),
                requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand"));

        this.outputModuleStates = requireNonNullParam(outputModuleStates, "frontLeftOutput", "SwerveControllerCommand");

        this.desiredRotation = requireNonNullParam(desiredRotation, "desiredRotation", "SwerveControllerCommand");

        addRequirements(requirements);
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
     * @param kinematics
     *            The kinematics for the robot drivetrain.
     * @param xController
     *            The Trajectory Tracker PID controller for the robot's x position.
     * @param yController
     *            The Trajectory Tracker PID controller for the robot's y position.
     * @param thetaController
     *            The Trajectory Tracker PID controller for angle for the robot.
     * @param outputModuleStates
     *            The raw output module states from the position controllers.
     * @param requirements
     *            The subsystems to require.
     */
    @SuppressWarnings("ParameterName")
    public SwerveControllerCommand(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Consumer<SwerveModuleState[]> outputModuleStates, DrivebaseSubsystem drivebase,
            Subsystem... requirements) {
        this(
                trajectory,
                pose,
                kinematics,
                xController,
                yController,
                thetaController,
                () -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
                outputModuleStates,
                drivebase, requirements);
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
        var targetModuleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);

        outputModuleStates.accept(targetModuleStates);
        // drivebase.drive(new Vector2(targetChassisSpeeds.vxMetersPerSecond,
        // targetChassisSpeeds.vyMetersPerSecond),
        // targetChassisSpeeds.omegaRadiansPerSecond, true);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Hurray it is called");
        if (interrupted) {
            // drivebase.drive(Vector2.ZERO, 0, false);
            outputModuleStates.accept(kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
        }
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
