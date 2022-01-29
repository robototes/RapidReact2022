// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot;

import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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

    public final Controls controls;
    public final Subsystems subsystems;
    public final Hardware hardware;
    
    private UpdateManager updateManager;

    private Robot() {
        instance = this;
        hardware = new Hardware();
        subsystems = new Subsystems(hardware);
        controls = new Controls(subsystems);
    }

    // TODO add other override methods

    @Override
    public void robotInit() {
        updateManager = new UpdateManager(
                subsystems.drivebaseSubsystem
        );
        updateManager.startLoop(5.0e-3);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        subsystems.drivebaseSubsystem.resetPose(RigidTransform2.ZERO);
        SimplePathBuilder builder = new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(0));
        subsystems.drivebaseSubsystem.follow(builder.lineTo(new Vector2(20, 0)).lineTo(new Vector2(20, 20)).lineTo(new Vector2(0, 20)).lineTo(new Vector2(0, 0)).build());
    }

}
