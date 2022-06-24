package frc.team2412.robot.util.MotorStuff;

import static com.ctre.phoenix.motorcontrol.ControlMode.*;
import static com.ctre.phoenix.motorcontrol.DemandType.*;
import static com.ctre.phoenix.motorcontrol.NeutralMode.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class FalconMotor extends WPI_TalonFX implements Motor {

    /**
     * Constructor for motor
     *
     * @param deviceNumber
     *            device ID of motor controller (this value is assigned in pheonix tuner)
     */
    public FalconMotor(int deviceNumber) {
        super(deviceNumber);
    }

    /**
     * Constructor for motor. Use in case the motor is on a different CAN network.
     * This is used primarily for any devices on a CANivore network
     *
     * @param deviceNumber
     *            device ID of motor controller (this value is assigned in pheonix tuner)
     * @param canbus
     *            Name of the CANbus (Canivore name can be found in pheonix tuner)
     *
     */
    public FalconMotor(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);
    }

    /**
     * Sets the motor to the target position.
     * Uses a PID loop and requires PID values to be nonzero
     *
     * @param value
     *            position setpoint for motor to go to. Units are in encoder ticks.
     */
    @Override
    public void setPosition(double value) {
        set(Position, value);
    }

    /**
     * Sets the motor to the target position using feedforward term as an offset to motor power.
     * Uses a PID loop and requires PID values to be nonzero.
     * Use this method when PID requires a constant force i.e. gravity compensation on climber or hood
     *
     * @param value
     *            position setpoint for motor to go to. Units are in encoder ticks.
     * @param feedforward
     *            feedforward value to be used as baseline motor power. Value must be between [-1,1]
     *
     * @see <a href=
     *      "https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#arbitrary-feed-forward">CTRE
     *      Documentation on feedforward</a>
     */
    @Override
    public void setPosition(double value, double feedforward) {
        set(Position, value, ArbitraryFeedForward, feedforward);
    }

    /**
     * Sets the motor to go target velocity.
     * Uses a PID loop and requires PID values to be nonzero
     *
     * @param value
     *            velocity setpoint for motor to go to. Units are in encoder ticks per 100ms.
     */
    @Override
    public void setVelocity(double value) {
        set(Velocity, value);
    }

    /**
     * Sets the motor to go target velocity using feedforward term as an offset to motor power.
     * Uses a PID loop and requires PID values to be nonzero.
     * Use this method when PID requires a constant force i.e. gravity compensation on climber or hood
     *
     * @param value
     *            velocity setpoint for motor to go to. Units are in encoder ticks per 100ms.
     * @param feedforward
     *            feedforward value to be used as baseline motor power. Value must be between [-1,1]
     *
     * @see <a href=
     *      "https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#arbitrary-feed-forward">CTRE
     *      Documentation on feedforward</a>
     */
    @Override
    public void setVelocity(double value, double feedforward) {
        set(Velocity, value, ArbitraryFeedForward, feedforward);
    }

    /**
     * Sets motor to coast, allowing motor to freely spin when acted upon other forces.
     * Motor will provide no resistance to rotation.
     * If wanting motor to stay in place, use brake mode
     */
    @Override
    public void setToCoastMode() {
        setNeutralMode(Coast);
    }

    /**
     * Sets motor to brake, causing motor to resist outside forces.
     * Using this will cause the motor to apply electrical resistance to rotation.
     * If wanting motor to spin freely, use coast mode
     */
    @Override
    public void setToBrakeMode() {
        setNeutralMode(Brake);
    }

    /**
     * Creates a current limit, preventing how much a motor can draw.
     *
     * @param amp
     *            maximum amount of amperage the motor can draw
     */
    @Override
    public void setCurrentLimit(double amp) {
        configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, amp, 0, 0));
    }

    /**
     * Creates a current limit, preventing how much a motor can draw.
     *
     * @param amp
     *            maximum amount of amperage the motor can draw
     * @param timeSeconds
     *            how long the motor can go above threshold before being limited
     */
    @Override
    public void setCurrentLimit(double amp, double timeSeconds) {
        configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, amp, amp, timeSeconds));
    }

    /**
     * Get the encoder position. This returns a value in encoder ticks. 2048 encoder ticks result in 1
     * motor revolution
     *
     * @return Position of encoder in encoder ticks.
     */
    @Override
    public double getEncoderPosition() {
        return getSelectedSensorPosition();
    }

    /**
     * Get the encoder velocity. This returns the rate in which the motor is spinning.
     * The value in encoder ticks per 100ms. 2048 encoder ticks result in 1 motor revolution
     *
     * @return velocity of encoder in encoder ticks.
     */
    @Override
    public double getEncoderVelocity() {
        return getSelectedSensorVelocity();
    }

    /**
     * Sets the encoder position to the given value.
     *
     * @param position
     *            Position to set for the selected sensor, value is in encoder ticks.
     */
    @Override
    public void setEncoderPosition(double position) {
        setSelectedSensorPosition(position);
    }

    /**
     * Sets the encoder position to 0.
     *
     */
    @Override
    public void resetEncoder() {
        setEncoderPosition(0);
    }

    /**
     * Sets the 'P' constant for a PID loop.
     * In a loop, this value is multiplied by closed loop error.
     *
     * @param value
     *            Value of the P constant.
     */
    @Override
    public void setP(double value) {
        config_kP(0, value);
    }

    /**
     * Sets the 'D' constant for a PID loop.
     *
     * This is multiplied by derivative error (slope of error over the last 1ms)
     *
     * @param value
     *            Value of the D constant.
     */
    @Override
    public void setD(double value) {
        config_kD(0, value);
    }

    /**
     * Sets the 'F' constant in PID loop.
     * This is a different value than arbitrary feedforward
     *
     * @see <a href=
     *      "https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html?highlight=kF#calculating-velocity-feed-forward-gain-kf">CTRE
     *      Documentation on F in PID</a>
     *
     *
     * @param value
     *            Value of the F constant.
     */
    @Override
    public void setF(double value) {
        config_kF(0, value);
    }

}
