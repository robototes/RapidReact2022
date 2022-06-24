package frc.team2412.robot.subsystem;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.*;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.sim.PhysicsSim;
import frc.team2412.robot.util.MotorStuff.FalconMotor;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.*;

public class ClimbSubsystem extends SubsystemBase implements Loggable {

    public static class ClimbConstants {
        // Climb dynamic motor speeds
        public static final double EXTEND_SPEED = 0.15;
        public static final double RETRACT_SPEED = -0.15;

        // PID stuff
        public static final int PID_EXTENSION_SLOT = 0;
        public static final double EXTENSION_P = 0.5;
        public static final double EXTENSION_I = 0;
        public static final double EXTENSION_D = 0;
        public static final double EXTENSION_F = 0;

        public static final int PID_RETRACTION_SLOT = 1;
        public static final double RETRACTION_P = 0.5; // TODO: figure out values
        public static final double RETRACTION_I = 0;
        public static final double RETRACTION_D = 0;
        public static final double RETRACTION_F = 0.18;
        // This is based on the minimum amount of motor power need to keep climb arm in
        // place, need to test

        // Relating to physical climb structure things
        // was previously mid
        public static final double MID_RUNG_HEIGHT = 5.5;
        public static final double RETRACT_HEIGHT = 0.166;

        public static final double CLIMB_OFFSET = 4.75;

        // Doing integer division, which returns 11757 (previously 8789)
        // Probably should do floating point division, which returns 11759.3
        public static final double ENCODER_TICKS_PER_REMY = ((272816.0 / 58) * 2 * 5) / 4 * 6;

        // Max robot height is 66 inches
        public static final double MAX_ENCODER_TICKS = (11 - CLIMB_OFFSET) * ENCODER_TICKS_PER_REMY;
        public static final double MIN_ENCODER_TICKS = 0;

        // Motor current limit config
        public static final SupplyCurrentLimitConfiguration MOTOR_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                true, 40, 60, 15);
    }

    @Log.MotorController
    private final FalconMotor motor;

    // For use in the simulationPeriodic to get encoder position
    private double lastUpdatedTime = Timer.getFPGATimestamp();

    public ClimbSubsystem() {
        setName("ClimbSubsystem");
        motor = new FalconMotor(CLIMB_FIXED_MOTOR);

        motor.setToBrakeMode();
        motor.setCurrentLimit(40, 15);

        motor.setP(EXTENSION_P);
        motor.setD(EXTENSION_D);

        setPIDRetract(RETRACTION_P, RETRACTION_I, RETRACTION_D);
    }

    /**
     * Graciously initialize the simulation, by setting
     * the initialization time of the motor (from 0 to full)
     * to 1 second, and setting the maximum velocity.
     *
     * @param sim
     *            The current gracious simulation
     */
    public void simInit(PhysicsSim sim) {
        // Motor, acceleration time from 0 to full in seconds, max velocity
        sim.addTalonFX(motor, 1, SIM_FULL_VELOCITY);
    }

    /**
     * Stop the dynamic climb arm motor, graciously
     *
     * @param stop
     *             Whether to stop the motor
     */
    @Config(name = "Stop Fixed Motor")
    public void stopArm(boolean stop) {
        if (stop) {
            motor.stopMotor();
        }
    }

    /**
     * Graciously extend the dynamic climb arm,
     * to the mid rung height
     */
    public void extendArm() {
        motor.selectProfileSlot(PID_EXTENSION_SLOT, 0);
        setMotor(MID_RUNG_HEIGHT * ENCODER_TICKS_PER_REMY, EXTENSION_F);
    }

    /**
     * Graciously retract the climb arm down the extra
     * amount needed to fully retract it.
     */
    public void retractArm() {
        motor.selectProfileSlot(PID_RETRACTION_SLOT, 0);
        setMotor(RETRACT_HEIGHT * ENCODER_TICKS_PER_REMY, RETRACTION_F);
    }

    /**
     * Set the position to which the motor will graciously follow
     *
     * @param value
     *              The position to set the motor
     */
    public void setMotor(double value, double feedforward) {
        motor.setPosition(value, feedforward);
    }

    public void setMotorSpeed(double speed) {
        motor.set(speed);
    }

    /**
     * Periodic function 🙊🙉🙈 in the simulation
     * Graciously updates the encoder position in the sim
     * calculating it with time and speed.
     */
    @Override
    public void simulationPeriodic() {
        // max speed 6000 rpm, 2048 ticks per rotation
        double timeNow = Timer.getFPGATimestamp();
        double timeElapsed = timeNow - lastUpdatedTime;
        double motorFixedSpeed = motor.getSelectedSensorVelocity();
        motor.getSimCollection().setIntegratedSensorRawPosition((int) (motorFixedSpeed / timeElapsed));
        lastUpdatedTime = timeNow;
    }

    /**
     * Get (graciously) the dynamic climb arm's encoder value
     *
     * @return the motor's encoder position
     */
    @Log
    public double encoderPosition() {
        return motor.getEncoderPosition();
    }

    /**
     * Get the height that has so graciously been climbed,
     * using the encoder position and offset 🤪
     *
     * @return the total climbed height in inches
     */
    @Log
    public double climbHeightRemy() {
        return encoderPosition() / ENCODER_TICKS_PER_REMY + CLIMB_OFFSET;
    }

    /**
     * Professionally set the encoder position to 0,
     * only if the reset value has been graciously
     * specified as true.
     *
     * @param reset
     *              whether to reset the encoder
     */
    @Config.ToggleButton
    public void resetEncoder(boolean reset) {
        if (reset) {
            motor.resetEncoder();
        }
    }

    /**
     * Configure the motor PID (probably graciously)
     *
     * @param p
     *          the P value to configure
     * @param i
     *          the I value to configure
     * @param d
     *          the D value to configure
     */
    @Config(name = "PID extend")
    private void setPIDExtend(@Config(name = "EXTENSION P", defaultValueNumeric = EXTENSION_P) double p,
            @Config(name = "EXTENSION I", defaultValueNumeric = EXTENSION_I) double i,
            @Config(name = "EXTENSION D", defaultValueNumeric = EXTENSION_D) double d) {
        motor.config_kP(PID_EXTENSION_SLOT, p);
        motor.config_kI(PID_EXTENSION_SLOT, i);
        motor.config_kD(PID_EXTENSION_SLOT, d);
    }

    @Config(name = "PID retract")
    private void setPIDRetract(@Config(name = "RETRACTION P", defaultValueNumeric = RETRACTION_P) double p,
            @Config(name = "RETRACTION I", defaultValueNumeric = RETRACTION_I) double i,
            @Config(name = "RETRACTION D", defaultValueNumeric = RETRACTION_D) double d) {
        motor.config_kP(PID_RETRACTION_SLOT, p);
        motor.config_kI(PID_RETRACTION_SLOT, i);
        motor.config_kD(PID_RETRACTION_SLOT, d);
    }

    /**
     * Graciously lowers arm at set retract speed
     */
    public void lowerArm() {
        motor.set(RETRACT_SPEED);
    }

    /**
     * Graciously extends arm at set speed
     */
    public void extendArmSlowly() {
        motor.set(EXTEND_SPEED);
    }

    /**
     * Determine whether the limit switch has been triggered
     *
     * @return whether the limit switch has been hit
     */
    public boolean isHittingLimitSwitch() {
        return true;
    }

}
