package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.*;
import static frc.team2412.robot.Hardware.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.sim.PhysicsSim;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ClimbSubsystem extends SubsystemBase implements Loggable {

    public static class ClimbConstants {
        // Climb dynamic motor speeds
        public static final double EXTEND_SPEED = 0.15;
        public static final double RETRACT_SPEED = -0.15;
        public static final double CLIMB_SPEED = 0.4;

        // Doing integer division, which returns 11757 (previously 8789)
        // Probably should do floating point division, which returns 11759.3
        public static final double ENCODER_TICKS_PER_INCH = ((272816.0 / 58) * 2 * 5) / 4;
        public static final double CLIMB_OFFSET_INCHES = 28.5;

        // Max robot height is 66 inches
        public static final double MAX_ENCODER_TICKS = (66 - CLIMB_OFFSET_INCHES) * ENCODER_TICKS_PER_INCH;
        public static final double MIN_ENCODER_TICKS = 0;

        // PID stuff
        public static final int PID_SLOT_0 = 0;
        public static final double P = 0.5;
        public static final double I = 0;
        public static final double D = 0;

        // Relating to physical climb structure things
        public static final double MID_RUNG_HEIGHT_INCH = 32;
        public static final double RETRACT_HEIGHT_INCH = 15;
        public static final double FULL_RETRACT_HEIGHT_INCH = 1;

        // Motor current limit config
        public static final SupplyCurrentLimitConfiguration MOTOR_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                true, 40, 60, 15);
    }

    @Log.MotorController
    private final WPI_TalonFX motor;
    private final DigitalInput bottomLimitSwitch;

    // For use in the simulationPeriodic to get encoder position
    private double lastUpdatedTime = Timer.getFPGATimestamp();

    public ClimbSubsystem() {
        setName("ClimbSubsystem");
        motor = new WPI_TalonFX(CLIMB_FIXED_MOTOR);
        bottomLimitSwitch = new DigitalInput(CLIMB_LIMIT_SWITCH);

        // Configure motor soft limits, current limits and peak outputs
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.forwardSoftLimitEnable = false;
        motorConfig.reverseSoftLimitEnable = false;
        motorConfig.forwardSoftLimitThreshold = MAX_ENCODER_TICKS;
        motorConfig.reverseSoftLimitThreshold = MIN_ENCODER_TICKS;
        motorConfig.supplyCurrLimit = MOTOR_CURRENT_LIMIT;
        motorConfig.peakOutputForward = CLIMB_SPEED;
        motorConfig.peakOutputReverse = -CLIMB_SPEED;
        motor.configAllSettings(motorConfig);
        motor.setNeutralMode(NeutralMode.Brake);

        setPID(P, I, D);
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
     *            Whether to stop the motor
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
        setMotor(MID_RUNG_HEIGHT_INCH * ENCODER_TICKS_PER_INCH);
    }

    /**
     * Retract the dynamic climb arm, professionally,
     * going down the RETRACT_HEIGHT_INCH value (most of the way)
     */
    public void retractArm() {
        setMotor(RETRACT_HEIGHT_INCH * ENCODER_TICKS_PER_INCH);
    }

    /**
     * Graciously retract the climb arm down the extra
     * amount needed to fully retract it.
     */
    public void retractArmFully() {
        setMotor(FULL_RETRACT_HEIGHT_INCH * ENCODER_TICKS_PER_INCH);
    }

    /**
     * Set the position to which the motor will graciously follow
     *
     * @param value
     *            The position to set the motor
     */
    public void setMotor(double value) {
        motor.set(ControlMode.Position, value);
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
        return motor.getSelectedSensorPosition();
    }

    /**
     * Get the height that has so graciously been climbed,
     * using the encoder position and offset 🤪
     *
     * @return the total climbed height in inches
     */
    @Log
    public double climbHeightInches() {
        return encoderPosition() / ENCODER_TICKS_PER_INCH + CLIMB_OFFSET_INCHES;
    }

    /**
     * Professionally set the encoder position to 0,
     * only if the reset value has been graciously
     * specified as true.
     *
     * @param reset
     *            whether to reset the encoder
     */
    @Config
    public void resetEncoder(boolean reset) {
        if (reset) {
            motor.setSelectedSensorPosition(0);
        }
    }

    /**
     * Configure the motor PID (probably graciously)
     *
     * @param p
     *            the P value to configure
     * @param i
     *            the I value to configure
     * @param d
     *            the D value to configure
     */
    @Config(name = "PID")
    private void setPID(@Config(name = "P", defaultValueNumeric = P) double p,
            @Config(name = "I", defaultValueNumeric = I) double i,
            @Config(name = "D", defaultValueNumeric = D) double d) {
        motor.config_kP(PID_SLOT_0, p);
        motor.config_kI(PID_SLOT_0, i);
        motor.config_kD(PID_SLOT_0, d);
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
        return bottomLimitSwitch != null ? bottomLimitSwitch.get() : true;
    }
}
