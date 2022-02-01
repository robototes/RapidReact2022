// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot;

import com.sun.jdi.Field;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2412.robot.util.GeoConvertor;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.List;

public class Robot extends TimedRobot {
    /**
     * Singleton Stuff
     */
    private static Robot instance = null;

    public static Robot getInstance() {
        if (instance == null)
            instance = new Robot();
        return instance;
    }

    public Controls controls;
    public Subsystems subsystems;
    public Hardware hardware;

    private UpdateManager updateManager;

    private Robot() {
        instance = this;
    }

    // TODO add other override methods

    public Trajectory trajectory;
    public Field2d field;
    @Override
    public void robotInit() {
        hardware = new Hardware();
        subsystems = new Subsystems(hardware);
        controls = new Controls(subsystems);
        updateManager = new UpdateManager(
                subsystems.drivebaseSubsystem);
        updateManager.startLoop(5.0e-3);

        // Create the trajectory to follow in autonomous. It is best to initialize
        // trajectories here to avoid wasting time in autonomous. This is an example trajectory, you do not need to
        //to have it, just set trajectory to debug
        trajectory =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
                        new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

        // Create and push Field2d to SmartDashboard.
        field = new Field2d();
        SmartDashboard.putData(field);

        // Push the trajectory to Field2d.
        //field.getObject("traj").setTrajectory(trajectory);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        //subsystems.drivebaseSubsystem.resetPose(new Pose2d(RigidTransform2.ZERO) );
        subsystems.drivebaseSubsystem.resetPose(GeoConvertor.rigidToPose(RigidTransform2.ZERO));
        SimplePathBuilder builder = new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(0));
        subsystems.drivebaseSubsystem.follow(builder.lineTo(new Vector2(20, 0)).lineTo(new Vector2(20, 20))
                .lineTo(new Vector2(0, 20)).lineTo(new Vector2(0, 0)).build());
    }

}
