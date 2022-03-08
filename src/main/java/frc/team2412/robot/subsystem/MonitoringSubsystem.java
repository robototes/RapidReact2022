package frc.team2412.robot.subsystem;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MonitoringSubsystem extends SubsystemBase {
    private final ShuffleboardTab tab = Shuffleboard.getTab("Power Monitoring");
    private final NetworkTableEntry totalPower, totalCurrent, currentTemperature,
            channel0current, channel1current, channel2current, channel3current, channel4current, channel5current,
            channel6current, channel7current,
            channel8current, channel9current, channel10current, channel11current, channel12current, channel13current,
            channel14current, channel15current;
    private final PowerDistribution powerDistributionPanel;

    public MonitoringSubsystem(PowerDistribution powerDistributionPanel) {
        this.powerDistributionPanel = powerDistributionPanel;

        totalPower = tab.add("Total Power", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
        totalCurrent = tab.add("Total Current", 0.0).withPosition(0, 1).withSize(1, 1).getEntry();
        currentTemperature = tab.add("Current Temperature", 0.0).withPosition(0, 2).withSize(1, 1).getEntry();
        channel0current = tab.add("Channel 0 Current", 0.0).withPosition(1, 0).withSize(1, 1).getEntry();
        channel1current = tab.add("Channel 1 Current", 0.0).withPosition(1, 1).withSize(1, 1).getEntry();
        channel2current = tab.add("Channel 2 Current", 0.0).withPosition(1, 2).withSize(1, 1).getEntry();
        channel3current = tab.add("Channel 3 Current", 0.0).withPosition(2, 0).withSize(1, 1).getEntry();
        channel4current = tab.add("Channel 4 Current", 0.0).withPosition(2, 1).withSize(1, 1).getEntry();
        channel5current = tab.add("Channel 5 Current", 0.0).withPosition(2, 2).withSize(1, 1).getEntry();
        channel6current = tab.add("Channel 6 Current", 0.0).withPosition(3, 0).withSize(1, 1).getEntry();
        channel7current = tab.add("Channel 7 Current", 0.0).withPosition(3, 1).withSize(1, 1).getEntry();
        channel8current = tab.add("Channel 8 Current", 0.0).withPosition(3, 2).withSize(1, 1).getEntry();
        channel9current = tab.add("Channel 9 Current", 0.0).withPosition(4, 0).withSize(1, 1).getEntry();
        channel10current = tab.add("Channel 10 Current", 0.0).withPosition(4, 1).withSize(1, 1).getEntry();
        channel11current = tab.add("Channel 11 Current", 0.0).withPosition(4, 2).withSize(1, 1).getEntry();
        channel12current = tab.add("Channel 12 Current", 0.0).withPosition(5, 0).withSize(1, 1).getEntry();
        channel13current = tab.add("Channel 13 Current", 0.0).withPosition(5, 1).withSize(1, 1).getEntry();
        channel14current = tab.add("Channel 14 Current", 0.0).withPosition(5, 2).withSize(1, 1).getEntry();
        channel15current = tab.add("Channel 15 Current", 0.0).withPosition(6, 0).withSize(1, 1).getEntry();
    }

    @Override
    public void periodic() {
        totalPower.setDouble(powerDistributionPanel.getTotalPower());
        totalCurrent.setDouble(powerDistributionPanel.getTotalCurrent());
        currentTemperature.setDouble(powerDistributionPanel.getTemperature());
        channel0current.setDouble(powerDistributionPanel.getCurrent(0));
        channel1current.setDouble(powerDistributionPanel.getCurrent(1));
        channel2current.setDouble(powerDistributionPanel.getCurrent(2));
        channel3current.setDouble(powerDistributionPanel.getCurrent(3));
        channel4current.setDouble(powerDistributionPanel.getCurrent(4));
        channel5current.setDouble(powerDistributionPanel.getCurrent(5));
        channel6current.setDouble(powerDistributionPanel.getCurrent(6));
        channel7current.setDouble(powerDistributionPanel.getCurrent(7));
        channel8current.setDouble(powerDistributionPanel.getCurrent(8));
        channel9current.setDouble(powerDistributionPanel.getCurrent(9));
        channel10current.setDouble(powerDistributionPanel.getCurrent(10));
        channel11current.setDouble(powerDistributionPanel.getCurrent(11));
        channel12current.setDouble(powerDistributionPanel.getCurrent(12));
        channel13current.setDouble(powerDistributionPanel.getCurrent(13));
        channel14current.setDouble(powerDistributionPanel.getCurrent(14));
        channel15current.setDouble(powerDistributionPanel.getCurrent(15));
        /*
         * Shouldn't use System.out.println()
         * Just in case the Shuffleboard not working
         * Will change to Shuffleboard later
         */
        System.out.println("Temperature: " + powerDistributionPanel.getTemperature());
        System.out.println("Total Power: " + powerDistributionPanel.getTotalPower());
        System.out.println("Total Current: " + powerDistributionPanel.getTotalCurrent());
    }
}
