// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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

    private UpdateManager updateManager;
    private Thread controlAuto;
    final private RobotType robotType;

    Robot(RobotType type) {
        instance = this;
        robotType = type;
    }

    // TODO add other override methods

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

        if (robotType.equals(RobotType.AUTOMATED_TEST)) {
            controlAuto = new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        sleep(2000);
                    } catch (InterruptedException ignored) {
                    }

                    DriverStationDataJNI.setAutonomous(true);
                    DriverStationDataJNI.setEnabled(true);

                    try {
                        sleep(10000);
                    } catch (InterruptedException ignored) {
                    }

                    DriverStationDataJNI.setEnabled(false);

                    try {
                        sleep(2000);
                    } catch (InterruptedException ignored) {
                    }

                    endCompetition();
                }
            });
            controlAuto.start();
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        subsystems.drivebaseSubsystem.resetPose(RigidTransform2.ZERO);
        SimplePathBuilder builder = new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(0));
        subsystems.drivebaseSubsystem.follow(builder.lineTo(new Vector2(20, 0)).lineTo(new Vector2(20, 20))
                .lineTo(new Vector2(0, 20)).lineTo(new Vector2(0, 0)).build());
    }

}
