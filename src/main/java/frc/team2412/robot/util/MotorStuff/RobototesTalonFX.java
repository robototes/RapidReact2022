package frc.team2412.robot.util.MotorStuff;

import static com.ctre.phoenix.motorcontrol.ControlMode.*;
import static com.ctre.phoenix.motorcontrol.DemandType.*;
import static com.ctre.phoenix.motorcontrol.NeutralMode.*;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class RobototesTalonFX extends WPI_TalonFX implements Motor {

    public RobototesTalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    public RobototesTalonFX(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);
    }

    @Override
    public void setPosition(double value) {
        set(Position, value);
    }

    @Override
    public void setVelocity(double value) {
        set(Velocity, value);
    }

    @Override
    public void setVelocity(double value, double feedforward) {
        set(Velocity, value, ArbitraryFeedForward, feedforward);
    }

    @Override
    public void setToCoastMode() {
        setNeutralMode(Coast);
    }

    @Override
    public void setToBrakeMode() {
        setNeutralMode(Brake);
    }

    @Override
    public void setCurrentLimit(double amp) {
        setCurrentLimit(amp, 0);
    }

    @Override
    public void setCurrentLimit(double amp, double timeSeconds) {
        configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, amp, 0, timeSeconds));
    }

    @Override
    public double getEncoderPosition() {
        return getSelectedSensorPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return getSelectedSensorVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        setSelectedSensorPosition(position);
    }

    @Override
    public void resetEncoder() {
        setEncoderPosition(0);
    }

    @Override
    public void setP(double value) {
        config_kP(0, value);
    }

    @Override
    public void setI(double value) {
        config_kI(0, value);
    }

    @Override
    public void setD(double value) {
        config_kD(0, value);
    }

    @Override
    public void setF(double value) {
        config_kF(0, value);
    }

}
