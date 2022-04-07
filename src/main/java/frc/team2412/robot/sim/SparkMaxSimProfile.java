package frc.team2412.robot.sim;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Simulation profile for a CANSparkMax.
 *
 * <p>Using PhysicsSim.addSparkMax() is recommended over manually creating sim profiles and adding
 * them to the sim.
 */
public class SparkMaxSimProfile extends SimProfile {
  public static class SparkMaxConstants {
    // Info is from https://www.revrobotics.com/rev-21-1651/
    public static final double STALL_TORQUE = 0.97;
    public static final double FREE_SPEED_RPM = 11000;
  }

  // Note: REVPhysicsSim is supposed to have a singleton, but instantiating separate instances
  // works.
  // Each sim has a separate ArrayList, though, so having separate sims with only one motor is
  // inefficient. The reason to have separate instances for each sim profile is so that running
  // one
  // won't affect others.
  private final REVPhysicsSim sim = new REVPhysicsSim();

  public SparkMaxSimProfile(CANSparkMax spark, double stallTorque, double freeSpeed) {
    sim.addSparkMax(spark, (float) stallTorque, (float) freeSpeed);
  }

  public SparkMaxSimProfile(CANSparkMax spark, DCMotor motor) {
    sim.addSparkMax(spark, motor);
  }

  @Override
  public void run() {
    sim.run();
  }
}
