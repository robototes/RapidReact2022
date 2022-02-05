// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.commands.autonomous.AutonomousCommand;
import frc.team2412.robot.util.Constants;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.robot.UpdateManager;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.util.AutonomousChooser;
import frc.team2412.robot.util.AutonomousTrajectories;
import frc.team2412.robot.subsystem.TestingSubsystem;

import static java.lang.Thread.sleep;

public class Robot extends TimedRobot {
    /**
     * Singleton Stuff
     */
    private static Robot instance = null;

    enum RobotType {
        COMPETITION, AUTOMATED_TEST
    }

    public static Robot getInstance(RobotType type) {
        if (instance == null)
            instance = new Robot(type);
        return instance;
    }

    public Controls controls;
    public Subsystems subsystems;
    public Hardware hardware;
    private Command autonomousCommand;

    private UpdateManager updateManager;
    private AutonomousChooser autonomousChooser;
    final private RobotType robotType;

    private Thread controlAuto;

    public TestingSubsystem testingSubsystem;

    Robot(RobotType type) {
        System.out.println("Robot type: " + (type.equals(RobotType.AUTOMATED_TEST) ? "AutomatedTest" : "Competition"));
        instance = this;
        robotType = type;
    }

    // TODO add other override methods

    public Trajectory trajectory;
    public Field2d field;

    @Override
    public void startCompetition() {
        if (!robotType.equals(RobotType.AUTOMATED_TEST)) {
            super.startCompetition();
        } else {
            try {
                super.startCompetition();
            } catch (Throwable throwable) {
                Throwable cause = throwable.getCause();
                if (cause != null) {
                    // We're about to exit, so overwriting the param is fine
                    // noinspection AssignmentToCatchBlockParameter
                    throwable = cause;
                }
                DriverStation.reportError(
                        "Unhandled exception: " + throwable.toString(), throwable.getStackTrace());

                try {
                    sleep(2000);
                } catch (InterruptedException ignored) {
                }
                java.lang.System.exit(-1);
            }
        }
    }

    @Override
    public void robotInit() {
        hardware = new Hardware();
        subsystems = new Subsystems(hardware);
        controls = new Controls(subsystems);
        updateManager = new UpdateManager(
                subsystems.drivebaseSubsystem);
        updateManager.startLoop(5.0e-3);

        // Create the trajectory to follow in autonomous. It is best to initialize
        // trajectories here to avoid wasting time in autonomous. This is an example
        // trajectory, you do not need to
        // to have it, just set trajectory to debug
        // trajectory =
        // TrajectoryGenerator.generateTrajectory(
        // new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        // new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

        // Create and push Field2d to SmartDashboard.
        field = new Field2d();
        SmartDashboard.putData(field);
        // field.getObject("traj").setTrajectory(trajectory);

        // Push the trajectory to Field2d.
        // field.getObject("traj").setTrajectory(trajectory);
        autonomousChooser = new AutonomousChooser(
                new AutonomousTrajectories(DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS));

        if (robotType.equals(RobotType.AUTOMATED_TEST)) {
            controlAuto = new Thread(new Runnable() {
                @Override
                public void run() {
                    System.out.println("Waiting two seconds for robot to finish startup");
                    try {
                        sleep(2000);
                    } catch (InterruptedException ignored) {
                    }

                    System.out.println("Enabling autonomous mode and waiting 10 seconds");
                    DriverStationDataJNI.setAutonomous(true);
                    DriverStationDataJNI.setEnabled(true);

                    try {
                        sleep(10000);
                    } catch (InterruptedException ignored) {
                    }

                    System.out.println("Disabling robot and waiting two seconds");
                    DriverStationDataJNI.setEnabled(false);

                    try {
                        sleep(2000);
                    } catch (InterruptedException ignored) {
                    }

                    System.out.println("Ending competition");
                    suppressExitWarning(true);
                    endCompetition();
                }
            });
            controlAuto.start();
        }
    }

    @Override
    public void testInit() {
        testingSubsystem = new TestingSubsystem();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.maxSpeedMetersPerSecond,
                Constants.AutoConstants.maxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.driveKinematics);

        // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory =
        // TrajectoryGenerator.generateTrajectory(
        // // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // // Pass through these two interior waypoints, making an 's' curve path
        // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // // End 3 meters straight ahead of where we started, facing forward
        // new Pose2d(3, 0, new Rotation2d(0)),
        // config);

        // System.out.println(exampleTrajectory.getStates());

        // System.out.println(exampleTrajectory.getTotalTimeSeconds());

    }

    @Override
    public void autonomousInit() {
        subsystems.drivebaseSubsystem.resetPose(RigidTransform2.ZERO);

        autonomousChooser.getCommand(subsystems).schedule();
        // subsystems.drivebaseSubsystem.resetPose(new Pose2d(RigidTransform2.ZERO) );
        autonomousCommand = new AutonomousCommand(subsystems.drivebaseSubsystem).getAutonomousCommand();
        autonomousCommand.schedule();
    }
    @Override public void autonomousExit(){
        if (autonomousCommand !=null){
            autonomousCommand.cancel();
        }
    }

}
