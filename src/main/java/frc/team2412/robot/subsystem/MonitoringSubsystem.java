package frc.team2412.robot.subsystem;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MonitoringSubsystem extends SubsystemBase {
    private ShuffleboardTab tab = Shuffleboard.getTab("Power Monitoring");
    private NetworkTableEntry totalPower;
    private PowerDistribution powerDistributionPanel;

    public MonitoringSubsystem(PowerDistribution powerDistributionPanel) {
        this.powerDistributionPanel = powerDistributionPanel;

        totalPower = tab.add("Total Power", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
    }

    @Override
    public void periodic() {
        totalPower.setDouble(powerDistributionPanel.getTotalPower());
        /*
         * Shouldn't use System.out.println()
         * Will change to Shuffleboard later
         */
        System.out.println("Temperature: " + powerDistributionPanel.getTemperature());
        System.out.println("Total Power: " + powerDistributionPanel.getTotalPower());
        System.out.println("Total Current: " + powerDistributionPanel.getTotalCurrent());
    }

}
