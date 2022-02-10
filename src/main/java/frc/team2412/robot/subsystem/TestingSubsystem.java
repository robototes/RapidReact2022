package frc.team2412.robot.subsystem;

import java.time.Duration;
import java.time.Instant;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestingSubsystem extends SubsystemBase {
    ShuffleboardTab tab = Shuffleboard.getTab("Testing Hardware");
    NetworkTableEntry timeDuration, redValue, greenValue, blueValue, proximityValue;
    ColorSensorV3 colorSensor;

    /**
     * This testing subsystem is intend for testing new hardware,
     * and should not exist on the final robot
     * Since the robotInit() is being called on the first place anyway,
     */
    public TestingSubsystem() {
        timeDuration = tab.add("Time Duration", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        proximityValue = tab.add("DIstance", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        redValue = tab.add("Red", 0.0)
                .withPosition(1, 0)
                .withSize(1, 1)
                .getEntry();
        greenValue = tab.add("Green", 0.0)
                .withPosition(1, 1)
                .withSize(1, 1)
                .getEntry();
        blueValue = tab.add("Blue", 0.0)
                .withPosition(1, 2)
                .withSize(1, 1)
                .getEntry();

        /*
         * The testing subsystem have its built-in Hardware class,
         * that no need to initialize and store the Hardware in somewhere else
         */
        this.colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    }

    @Override
    public void periodic() {
        /*
         * to measure how long does the ColorSensorV3 take to measure once
         * remember only send to value to telemetry outside the time-evaluation block
         */
        Instant start = Instant.now();
        double rValue = colorSensor.getRed();
        double gValue = colorSensor.getGreen();
        double bValue = colorSensor.getBlue();
        double pValue = colorSensor.getProximity();
        Duration timeElapsed = Duration.between(start, Instant.now());
        redValue.setDouble(rValue);
        greenValue.setDouble(gValue);
        blueValue.setDouble(bValue);
        proximityValue.setDouble(pValue);
        timeDuration.setDouble(timeElapsed.toNanos() / 1000);
    }
}
