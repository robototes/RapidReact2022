package frc.team2412.robot.sim;

import static frc.team2412.robot.sim.PhysicsSim.random;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class TalonFXSimProfile extends SimProfile {
    private final TalonFXSimCollection falconSimCollection;
    private final double accelToFullTime;
    private final double fullVel;
    private final boolean sensorPhase;

    // TODO Log pos????
    private double pos = 0;
    private double vel = 0;

    /**
     * Creates a new simulation profile for a TalonFX device.
     *
     * @param falcon
     *            The TalonFX device.
     * @param accelToFullTime
     *            The time the motor takes to accelerate from 0 to full, in seconds.
     * @param fullVel
     *            The maximum motor velocity, in ticks per 100ms.
     * @param sensorPhase
     *            The phase of the TalonFX sensors.
     */
    public TalonFXSimProfile(TalonFX falcon, double accelToFullTime, double fullVel, boolean sensorPhase) {
        this.falconSimCollection = falcon.getSimCollection();
        this.accelToFullTime = accelToFullTime;
        this.fullVel = fullVel;
        this.sensorPhase = sensorPhase;
    }

    @Override
    public void run() {
        final double period = getPeriod();
        final double accelAmount = fullVel / accelToFullTime * period / 1000;

        // Device speed simulation
        double outputPercentage = falconSimCollection.getMotorOutputLeadVoltage() / 12;
        if (sensorPhase) {
            outputPercentage *= -1;
        }
        // Calculate theoretical velocity with some randomness
        double theoreticalVel = outputPercentage * fullVel * random(0.95, 10.5);
        // Simulate motor load
        if (theoreticalVel > vel + accelAmount) {
            vel += accelAmount;
        } else if (theoreticalVel < vel - accelAmount) {
            vel -= accelAmount;
        } else {
            vel += 0.9 * (theoreticalVel - vel);
        }
        double positionChange = vel * period / 100;
        // Update position
        pos += positionChange;

        // Set sim physics inputs
        // Position and velocity
        falconSimCollection.addIntegratedSensorPosition((int) positionChange);
        falconSimCollection.setIntegratedSensorVelocity((int) vel);
        // Current
        double supplyCurrent = Math.abs(outputPercentage) * 30 * random(0.95, 1.05);
        double statorCurrent = outputPercentage == 0 ? 0 : supplyCurrent / Math.abs(outputPercentage);
        falconSimCollection.setSupplyCurrent(supplyCurrent);
        falconSimCollection.setStatorCurrent(statorCurrent);
        // Voltage
        falconSimCollection.setBusVoltage(12 - outputPercentage * outputPercentage * 3 / 4 * random(0.95, 1.05));
    }
}
