package frc.team2412.robot.subsystem;

import java.time.Duration;
import java.time.Instant;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestingSubsystem extends SubsystemBase{
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    NetworkTableEntry sensorValue, timeDuration;
    ColorSensorV3 colorSensor;

    public TestingSubsystem(ColorSensorV3 cs){
        sensorValue = tab.add("Sensor Value", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        timeDuration = tab.add("Time Duration", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        this.colorSensor = cs;
    }

    @Override
    public void periodic(){
        Instant start = Instant.now();
        sensorValue.setDouble(colorSensor.getRed());
        Duration timeElapsed = Duration.between(start, Instant.now());
        timeDuration.setDouble(timeElapsed.toMillis());
    }
}
