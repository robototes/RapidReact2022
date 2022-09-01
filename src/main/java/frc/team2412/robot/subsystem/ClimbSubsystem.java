package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.Constants.ClimbConstants.*;
import static frc.team2412.robot.Hardware.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.sim.PhysicsSim;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ClimbSubsystem extends SubsystemBase implements Loggable {

    @Log.MotorController
    private final WPI_TalonFX motor;

    // For use in the simulationPeriodic to get encoder position
    private double lastUpdatedTime = Timer.getFPGATimestamp();

    public ClimbSubsystem() {
        setName("ClimbSubsystem");
        motor = new WPI_TalonFX(CLIMB_FIXED_MOTOR);

        motor.configSupplyCurrentLimit(MOTOR_CURRENT_LIMIT);
        motor.setNeutralMode(NeutralMode.Brake);

        setPIDExtend(EXTENSION_P, EXTENSION_I, EXTENSION_D);
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
     *            The position to set the motor
     */
    public void setMotor(double value, double feedForward) {
        motor.set(ControlMode.Position, value, DemandType.ArbitraryFeedForward, feedForward);
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
        return motor.getSelectedSensorPosition();
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
     *            whether to reset the encoder
     */
    @Config.ToggleButton
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
