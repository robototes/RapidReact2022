// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.robot.UpdateManager;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.util.AutonomousChooser;
import frc.team2412.robot.util.AutonomousTrajectories;
import frc.team2412.robot.subsystem.TestingSubsystem;

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
    private AutonomousChooser autonomousChooser;

    public TestingSubsystem testingSubsystem;

    private Robot() {
        instance = this;
    }

    // TODO add other override methods

    @Override
    public void robotInit() {
        hardware = new Hardware();
        subsystems = new Subsystems(hardware);
        controls = new Controls(subsystems);
        updateManager = new UpdateManager(
                subsystems.drivebaseSubsystem);
        updateManager.startLoop(5.0e-3);
        autonomousChooser = new AutonomousChooser(
                new AutonomousTrajectories(DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS));
    }

    @Override
    public void testInit() {
        testingSubsystem = new TestingSubsystem();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        subsystems.drivebaseSubsystem.resetPose(RigidTransform2.ZERO);

        autonomousChooser.getCommand(subsystems).schedule();
    }

}
