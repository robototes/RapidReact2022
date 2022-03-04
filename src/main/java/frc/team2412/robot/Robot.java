// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot;

import static java.lang.Thread.sleep;

import frc.team2412.robot.commands.shooter.ShooterResetEncodersCommand;
import static frc.team2412.robot.Subsystems.SubsystemConstants.*;
import frc.team2412.robot.sim.PhysicsSim;
import frc.team2412.robot.sim.SparkMaxSimProfile.SparkMaxConstants;
import frc.team2412.robot.sim.TalonFXSimProfile.TalonFXConstants;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.robot.UpdateManager;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2412.robot.Subsystems.SubsystemConstants;
import frc.team2412.robot.commands.drive.DriveCommand;
import frc.team2412.robot.subsystem.DrivebaseSubsystem;
import frc.team2412.robot.subsystem.TestingSubsystem;
import frc.team2412.robot.util.autonomous.AutonomousTrajectories;
import frc.team2412.robot.util.autonomous.AutonomousChooser;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;

import java.io.IOException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

public class Robot extends TimedRobot implements Loggable {
    /**
     * Singleton Stuff
     */
    private static Robot instance = null;

    enum RobotType {
        COMPETITION, AUTOMATED_TEST, DRIVEBASE;
    }

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
    final private RobotType robotType;

    private Thread controlAuto;

    public TestingSubsystem testingSubsystem;

    protected Robot(RobotType type) {
        System.out.println("Robot type: " + (type.equals(RobotType.AUTOMATED_TEST) ? "AutomatedTest" : "Competition"));
        instance = this;
        robotType = type;
    }

    protected Robot() {
        this(getTypeFromAddress());
    }

    private static final byte[] COMPETITION_BOT_MAC_ADDRESS = new byte[] {
            0x00, (byte) 0x80, 0x2f, 0x33, (byte) 0x9d, (byte) 0xe7
    };
    private static final byte[] PRACTICE_BOT_MAC_ADDRESS = new byte[] {
            0x00, (byte) 0x80, 0x2f, 0x28, 0x40, (byte) 0x82
    };

    private static RobotType getTypeFromAddress() {
        List<byte[]> macAddresses;
        try {
            macAddresses = getMacAddresses();
        } catch (IOException ignored) {
            macAddresses = List.of();
        }

        for (byte[] macAddress : macAddresses) {
            // First check if we are the competition bot
            if (Arrays.compare(COMPETITION_BOT_MAC_ADDRESS, macAddress) == 0) {
                return RobotType.COMPETITION;
            }

            // Next check if we are the practice bot
            if (Arrays.compare(PRACTICE_BOT_MAC_ADDRESS, macAddress) == 0) {
                return RobotType.DRIVEBASE;
            }
        }
        // lol fallback
        return RobotType.COMPETITION;
    }

    private static List<byte[]> getMacAddresses() throws IOException {
        List<byte[]> macAddresses = new ArrayList<>();

        Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();

        NetworkInterface networkInterface;
        while (networkInterfaces.hasMoreElements()) {
            networkInterface = networkInterfaces.nextElement();

            byte[] address = networkInterface.getHardwareAddress();
            if (address == null) {
                continue;
            }

            macAddresses.add(address);
        }

        return macAddresses;
    }

    // TODO add other override methods

    public Field2d field = new Field2d();

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
        if (SubsystemConstants.DRIVE_ENABLED) {
            updateManager = new UpdateManager(
                    subsystems.drivebaseSubsystem);
            updateManager.startLoop(5.0e-3);
        }

        // Create and push Field2d to SmartDashboard.
        SmartDashboard.putData(field);

        autonomousChooser = new AutonomousChooser(subsystems,
                new AutonomousTrajectories(DrivebaseSubsystem.DriveConstants.TRAJECTORY_CONSTRAINTS));
        Logger.configureLoggingAndConfig(subsystems, false);

        CommandScheduler.getInstance()
                .onCommandInitialize(
                        command -> System.out.println("Command initialized: " + command.getName()));
        CommandScheduler.getInstance()
                .onCommandInterrupt(
                        command -> System.out.println("Command interrupted: " + command.getName()));
        CommandScheduler.getInstance()
                .onCommandFinish(
                        command -> System.out.println("Command finished: " + command.getName()));

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
        autonomousChooser.getCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        Logger.updateEntries();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {

        if (SubsystemConstants.DRIVE_ENABLED) {
            subsystems.drivebaseSubsystem.resetPose(RigidTransform2.ZERO);
        }

        if (SubsystemConstants.SHOOTER_ENABLED && isCompetition()) {
            new ShooterResetEncodersCommand(subsystems.shooterSubsystem).schedule();
        }

        autonomousChooser.getCommand().schedule();
    }

    @Override
    public void teleopInit() {
        if (SubsystemConstants.DRIVE_ENABLED) {
            subsystems.drivebaseSubsystem.setDefaultCommand(new DriveCommand(subsystems.drivebaseSubsystem,
                    controls.driveController.getLeftXAxis(), controls.driveController.getLeftYAxis(),
                    controls.driveController.getRightXAxis()));
        }
    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationInit() {
        PhysicsSim physicsSim = PhysicsSim.getInstance();
        // TODO Find more accurate values
        if (CLIMB_ENABLED) {
            // Motor, acceleration time from 0 to full in seconds, max velocity
            physicsSim.addTalonFX(hardware.climbMotorFixed, 1, 6000 * TalonFXConstants.RPM_TO_VELOCITY);
            physicsSim.addTalonFX(hardware.climbMotorDynamic, 1, 6000 * TalonFXConstants.RPM_TO_VELOCITY);
        }
        if (INTAKE_ENABLED) {
            physicsSim.addTalonFX(hardware.intakeMotor, 1, 6000 * TalonFXConstants.RPM_TO_VELOCITY);
        }
        if (INDEX_ENABLED) {
            physicsSim.addTalonFX(hardware.ingestIndexMotor, 1, 6000 * TalonFXConstants.RPM_TO_VELOCITY);
            physicsSim.addTalonFX(hardware.feederIndexMotor, 1, 6000 * TalonFXConstants.RPM_TO_VELOCITY);
        }
        if (SHOOTER_ENABLED) {
            physicsSim.addTalonFX(hardware.flywheelMotor1, 3, 6000 * TalonFXConstants.RPM_TO_VELOCITY);
            physicsSim.addTalonFX(hardware.flywheelMotor2, 3, 6000 * TalonFXConstants.RPM_TO_VELOCITY);
            physicsSim.addTalonFX(hardware.turretMotor, 1, 6000 * TalonFXConstants.RPM_TO_VELOCITY);
            // Motor, stall torque, maximum free RPM
            physicsSim.addSparkMax(hardware.hoodMotor, SparkMaxConstants.STALL_TORQUE,
                    SparkMaxConstants.FREE_SPEED_RPM);
        }
    }

    @Override
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    public RobotType getRobotType() {
        return robotType;
    }

    public boolean isCompetition() {
        return getRobotType() == RobotType.COMPETITION || getRobotType() == RobotType.AUTOMATED_TEST;
    }
}
