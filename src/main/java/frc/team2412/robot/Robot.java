// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;
import static frc.team2412.robot.Hardware.*;

import static java.lang.Thread.sleep;

import frc.team2412.robot.commands.autonomous.JackFiveBallAutoCommand;
import org.frcteam2910.common.robot.UpdateManager;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2412.robot.sim.PhysicsSim;
import frc.team2412.robot.subsystem.TestingSubsystem;
import frc.team2412.robot.util.MACAddress;
import frc.team2412.robot.util.autonomous.AutonomousChooser;
import io.github.oblarg.oblog.Logger;

public class Robot extends TimedRobot {
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

    private final PowerDistribution PDP;
    private UsbCamera driverVisionCamera;
    private PneumaticHub pneumaticHub;

    private static final double MIN_PRESSURE = 90;
    private static final double MAX_PRESSURE = 110;

    public Controls controls;
    public Subsystems subsystems;

    private UpdateManager updateManager;
    private AutonomousChooser autonomousChooser;

    final private RobotType robotType;

    private Thread controlAuto;

    public TestingSubsystem testingSubsystem;

    protected Robot(RobotType type) {
        System.out.println("Robot type: " + (type.equals(RobotType.AUTOMATED_TEST) ? "AutomatedTest" : "Competition"));
        instance = this;
        PDP = new PowerDistribution(Hardware.PDP_ID, ModuleType.kRev);
        robotType = type;
    }

    public double getVoltage() {
        return PDP.getVoltage();
    }

    protected Robot() {
        this(getTypeFromAddress());
    }

    public static final MACAddress COMPEITION_ADDRESS = MACAddress.of(0x33, 0x9d, 0xe7);
    public static final MACAddress PRACTICE_ADDRESS = MACAddress.of(0x28, 0x40, 0x82);

    private static RobotType getTypeFromAddress() {
        if (PRACTICE_ADDRESS.exists())
            return RobotType.DRIVEBASE;
        else
            return RobotType.COMPETITION;
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
        LiveWindow.setEnabled(false);
        subsystems = new Subsystems();
        controls = new Controls(subsystems);
        if (DRIVE_ENABLED) {
            updateManager = new UpdateManager(
                    subsystems.drivebaseSubsystem);
            updateManager.startLoop(0.011); // 0.005 previously
        }
        if (DRIVER_VIS_ENABLED) {
            driverVisionCamera = new UsbCamera("Driver Vision Front", Hardware.FRONT_CAM);
            driverVisionCamera.setResolution(160, 90);
            CameraServer.addCamera(driverVisionCamera);
            CameraServer.startAutomaticCapture();
        }

        if (COMPRESSOR_ENABLED) {
            pneumaticHub = new PneumaticHub(PNEUMATIC_HUB);
            pneumaticHub.enableCompressorAnalog(MIN_PRESSURE, MAX_PRESSURE);
        } else {
            pneumaticHub = new PneumaticHub(PNEUMATIC_HUB);
            if (pneumaticHub != null) {
                pneumaticHub.disableCompressor();
            }
        }

        Shuffleboard.startRecording();

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog(), true);

        // Create and push Field2d to SmartDashboard.
        SmartDashboard.putData(field);

        autonomousChooser = new AutonomousChooser(subsystems);
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
            controlAuto = new Thread(() -> {
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
            });
            controlAuto.start();
        }
        JackFiveBallAutoCommand.FiveBallConstants.init();
    }

    @Override
    public void testInit() {
        autonomousChooser.scheduleCommand();
    }

    @Override
    public void robotPeriodic() {
        Logger.updateEntries();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {

        if (subsystems.drivebaseSubsystem != null) {
            subsystems.drivebaseSubsystem.resetPose(autonomousChooser.getStartPose());
        }

        // if (subsystems.shooterSubsystem != null) {
        // new ShooterResetEncodersCommand(subsystems.shooterSubsystem).schedule();
        // }

        autonomousChooser.scheduleCommand();
    }

    @Override
    public void teleopInit() {
        if (subsystems.intakeSubsystem != null) {
            subsystems.intakeSubsystem.intakeExtend();
        }

    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationInit() {
        PhysicsSim sim = PhysicsSim.getInstance();
        if (subsystems.climbSubsystem != null) {
            subsystems.climbSubsystem.simInit(sim);
        }
        if (subsystems.indexSubsystem != null) {
            subsystems.indexSubsystem.simInit(sim);
        }
        if (subsystems.intakeSubsystem != null) {
            subsystems.intakeSubsystem.simInit(sim);
        }
        if (subsystems.shooterSubsystem != null) {
            subsystems.shooterSubsystem.simInit(sim);
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

    @Override
    public void disabledInit() {
        if (subsystems.climbSubsystem != null) {
            subsystems.climbSubsystem.stopArm(true);
        }
        if (subsystems.shooterSubsystem != null) {
            subsystems.shooterSubsystem.stopHoodMotor();
            subsystems.shooterSubsystem.stopFlywheel();
        }
    }
}
