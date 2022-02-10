package frc.team2412.robot.subsystem;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MonitoringSubsystem extends SubsystemBase {
    ShuffleboardTab tab = Shuffleboard.getTab("Power Monitoring");
    NetworkTableEntry voltage;
    PowerDistribution powerDistributionPanel;

    public MonitoringSubsystem(PowerDistribution powerDistributionPanel) {
        this.powerDistributionPanel = powerDistributionPanel;

        voltage = tab.add("Voltage", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
    }

    @Override
    public void periodic() {
        voltage.setDouble(powerDistributionPanel.getVoltage());
    }

}
