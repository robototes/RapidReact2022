package frc.team2412.robot.subsystem;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.util.MultiplexedColorSensor;

public class TestingSubsystem extends SubsystemBase {
    ShuffleboardTab tab = Shuffleboard.getTab("Testing Hardware");
    NetworkTableEntry displayTimeDuration, displayRedValue, displayGreenValue, displayBlueValue;
    MultiplexedColorSensor colorSensor0, colorSensor1;

    /**
     * This testing subsystem is intend for testing new hardware,
     * and should not exist on the final robot
     * Since the robotInit() is being called on the first place anyway,
     */
    public TestingSubsystem() {
        displayTimeDuration = tab.add("Time Duration", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
        displayRedValue = tab.add("#0 Red Value", 0.0).withPosition(1, 0).withSize(1, 1).getEntry();
        displayGreenValue = tab.add("#1 Green Value", 0.0).withPosition(1, 1).withSize(1, 1).getEntry();
        displayBlueValue = tab.add("#0 Blue Value", 0.0).withPosition(1, 2).withSize(1, 1).getEntry();
        /*
         * The testing subsystem have its built-in Hardware class,
         * that no need to initialize and store the Hardware in somewhere else
         */
        this.colorSensor0 = new MultiplexedColorSensor(0);
        this.colorSensor1 = new MultiplexedColorSensor(1);
    }

    @Override
    public void periodic() {
        /*
         * to measure how long does the ColorSensorV3 take to measure once
         * remember only send to value to telemetry outside the time-evaluation block
         */
        Instant start = Instant.now();
        // get raw data start
        double ColorSensorRedValue = colorSensor0.getRed();
        double ColorSensorGreenValue = colorSensor1.getGreen();
        double ColorSensorBlueValue = colorSensor0.getBlue();
        // get raw data end
        Duration timeElapsed = Duration.between(start, Instant.now());
        // output to shuffleboard here
        displayTimeDuration.setDouble(timeElapsed.toNanos() / 1000f);
        displayRedValue.setDouble(ColorSensorRedValue);
        displayGreenValue.setDouble(ColorSensorGreenValue);
        displayBlueValue.setDouble(ColorSensorBlueValue);
    }
}