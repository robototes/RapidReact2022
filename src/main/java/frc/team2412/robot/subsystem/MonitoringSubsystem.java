package frc.team2412.robot.subsystem;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MonitoringSubsystem extends SubsystemBase {
    private final ShuffleboardTab tab = Shuffleboard.getTab("Power Monitoring");
    private final NetworkTableEntry totalPower, totalCurrent, currentTemperature, 
            channel0, channel1, channel2, channel3, channel4, channel5, channel6, channel7, 
            channel8, channel9, channel10, channel11, channel12, channel13, channel14, channel15;
    private final PowerDistribution powerDistributionPanel;

    public MonitoringSubsystem(PowerDistribution powerDistributionPanel) {
        this.powerDistributionPanel = powerDistributionPanel;

        totalPower = tab.add("Total Power", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
        totalCurrent = tab.add("Total Current", 0.0).withPosition(0, 1).withSize(1, 1).getEntry();
        currentTemperature = tab.add("Current Temperature", 0.0).withPosition(0, 2).withSize(1, 1).getEntry();
        channel0 = tab.add("Channel 0", 0.0).withPosition(1, 0).withSize(1, 1).getEntry();
        channel1 = tab.add("Channel 1", 0.0).withPosition(1, 1).withSize(1, 1).getEntry();
        channel2 = tab.add("Channel 2", 0.0).withPosition(1, 2).withSize(1, 1).getEntry();
        channel3 = tab.add("Channel 3", 0.0).withPosition(2, 0).withSize(1, 1).getEntry();
        channel4 = tab.add("Channel 4", 0.0).withPosition(2, 1).withSize(1, 1).getEntry();
        channel5 = tab.add("Channel 5", 0.0).withPosition(2, 2).withSize(1, 1).getEntry();
        channel6 = tab.add("Channel 6", 0.0).withPosition(3, 0).withSize(1, 1).getEntry();
        channel7 = tab.add("Channel 7", 0.0).withPosition(3, 1).withSize(1, 1).getEntry();
        channel8 = tab.add("Channel 8", 0.0).withPosition(3, 2).withSize(1, 1).getEntry();
        channel9 = tab.add("Channel 9", 0.0).withPosition(4, 0).withSize(1, 1).getEntry();
        channel10 = tab.add("Channel 10", 0.0).withPosition(4, 1).withSize(1, 1).getEntry();
        channel11 = tab.add("Channel 11", 0.0).withPosition(4, 2).withSize(1, 1).getEntry();
        channel12 = tab.add("Channel 12", 0.0).withPosition(5, 0).withSize(1, 1).getEntry();
        channel13 = tab.add("Channel 13", 0.0).withPosition(5, 1).withSize(1, 1).getEntry();
        channel14 = tab.add("Channel 14", 0.0).withPosition(5, 2).withSize(1, 1).getEntry();
        channel15 = tab.add("Channel 15", 0.0).withPosition(6, 0).withSize(1, 1).getEntry();
    }

    @Override
    public void periodic() {
        totalPower.setDouble(powerDistributionPanel.getTotalPower());
        totalCurrent.setDouble(powerDistributionPanel.getTotalCurrent());
        currentTemperature.setDouble(powerDistributionPanel.getTemperature());
        channel0.setDouble(powerDistributionPanel.getCurrent(0));
        channel1.setDouble(powerDistributionPanel.getCurrent(1));
        channel2.setDouble(powerDistributionPanel.getCurrent(2));
        channel3.setDouble(powerDistributionPanel.getCurrent(3));
        channel4.setDouble(powerDistributionPanel.getCurrent(4));
        channel5.setDouble(powerDistributionPanel.getCurrent(5));
        channel6.setDouble(powerDistributionPanel.getCurrent(6));
        channel7.setDouble(powerDistributionPanel.getCurrent(7));
        channel8.setDouble(powerDistributionPanel.getCurrent(8));
        channel9.setDouble(powerDistributionPanel.getCurrent(9));
        channel10.setDouble(powerDistributionPanel.getCurrent(10));
        channel11.setDouble(powerDistributionPanel.getCurrent(11));
        channel12.setDouble(powerDistributionPanel.getCurrent(12));
        channel13.setDouble(powerDistributionPanel.getCurrent(13));
        channel14.setDouble(powerDistributionPanel.getCurrent(14));
        channel15.setDouble(powerDistributionPanel.getCurrent(15));
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
