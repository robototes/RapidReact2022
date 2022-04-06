package frc.team2412.robot.sim;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class PhysicsSim {
    /**
     * Returns a random number between min and max with values near min and max having a higher
     * probability.
     *
     * @param min
     *            The minimum value produced.
     * @param max
     *            The maximum value produced.
     * @return A random value between min and max, inclusive at both ends.
     */
    public static double random(double min, double max) {
        return (max + min) / 2 + (max - min) / 2 * Math.sin(Math.random() * 2 * Math.PI);
        // CTRE example:
        // "scales a random domain of [0, 2pi] to [min, max] while prioritizing the peaks"
        // return (max - min) / 2 * Math.sin(Math.IEEEremainder(Math.random(), 2 * Math.PI)) + (max + min) /
        // 2;
    }

    /**
     * Returns a random number between 0 and max with values near 0 or max having a higher probability.
     *
     * @param max
     *            The maximum value produced.
     * @return A random value between 0 and max, inclusive at both ends.
     */
    public static double random(double max) {
        return random(0, max);
    }

    // Singleton stuff
    private static final PhysicsSim instance = new PhysicsSim();

    /**
     * Returns the robot simulator instance.
     *
     * @return The robot simulator instance
     */
    public static PhysicsSim getInstance() {
        return instance;
    }

    private final ArrayList<SimProfile> simProfiles = new ArrayList<>();

    // See SparkMaxSimProfile for details
    private final REVPhysicsSim revPhysicsSim = new REVPhysicsSim();

    /**
     * Adds a TalonFX controller to the simulator.
     *
     * @param falcon
     *            The TalonFX device to add.
     * @param accelToFullTime
     *            The time the motor takes to accelerate from 0 to full, in seconds.
     * @param fullVel
     *            The maximum motor velocity, in ticks per 100ms.
     */
    public void addTalonFX(TalonFX falcon, double accelToFullTime, double fullVel) {
        addTalonFX(falcon, accelToFullTime, fullVel, false);
    }

    /**
     * Adds a TalonFX controller to the simulator.
     *
     * @param falcon
     *            The TalonFX device to add.
     * @param accelToFullTime
     *            The time the motor takes to accelerate from 0 to full, in seconds.
     * @param fullVel
     *            The maximum motor velocity, in ticks per 100ms.
     * @param sensorPhase
     *            The phase of the TalonFX sensors
     */
    public void addTalonFX(TalonFX falcon, double accelToFullTime, double fullVel, boolean sensorPhase) {
        if (falcon != null) {
            TalonFXSimProfile simFalcon = new TalonFXSimProfile(falcon, accelToFullTime, fullVel, sensorPhase);
            simProfiles.add(simFalcon);
        }
    }

    /**
     * Adds a {@link SimProfile} to the simulator.
     *
     * @param simProfile
     *            The {@link SimProfile} to add.
     */
    public void addSimProfile(SimProfile simProfile) {
        if (simProfile != null) {
            simProfiles.add(simProfile);
        }
    }

    /**
     * Adds a SparkMAX controller to the simulator.
     *
     * @param spark
     *            The SparkMAX device to add.
     * @param stallTorque
     *            The stall torque of the motor connected to SparkMAX (units are N m).
     * @param freeSpeed
     *            The maximum free speed in RPM.
     */
    public void addSparkMax(CANSparkMax spark, double stallTorque, double freeSpeed) {
        revPhysicsSim.addSparkMax(spark, (float) stallTorque, (float) freeSpeed);
    }

    /**
     * Adds a SparkMax controller to the simulator.
     *
     * @param spark
     *            The SparkMAX device to add.
     * @param motor
     *            The motor connected to the SparkMAX.
     */
    public void addSparkMax(CANSparkMax spark, DCMotor motor) {
        revPhysicsSim.addSparkMax(spark, motor);
    }

    /**
     * Runs the simulator.
     */
    public void run() {
        // Run each sim profile
        for (SimProfile simProfile : simProfiles) {
            simProfile.run();
        }
        revPhysicsSim.run();
    }
}
