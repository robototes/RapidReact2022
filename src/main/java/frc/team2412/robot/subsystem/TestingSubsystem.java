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
    NetworkTableEntry sensorValue, timeDuration;
    ColorSensorV3 colorSensor;

    /**
     * This testing subsystem is intend for testing new hardware,
     * and should not exist on the final robot
     * Since the robotInit() is being called on the first place anyway,
     * so as mentioned above this subsystem is only for hardware on top of existing hardware
     */
    public TestingSubsystem() {
        sensorValue = tab.add("Sensor Value", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        timeDuration = tab.add("Time Duration", 0.0)
                .withPosition(0, 1)
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
        double ColorSensorRedValue = colorSensor.getRed();
        Duration timeElapsed = Duration.between(start, Instant.now());
        sensorValue.setDouble(ColorSensorRedValue);
        timeDuration.setDouble(timeElapsed.toMillis());
    }
}
