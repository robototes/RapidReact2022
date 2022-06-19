package frc.team2412.robot.util.MotorStuff;

import com.revrobotics.CANSparkMax;

public class RobototesCANSparkMax extends CANSparkMax implements Motor {

    public RobototesCANSparkMax(int deviceId) {
        super(deviceId, MotorType.kBrushless);
    }

    @Override
    public void setPosition(double value) {
        getPIDController().setReference(value, ControlType.kPosition);
    }

    @Override
    public void setVelocity(double value) {
        getPIDController().setReference(value, ControlType.kVelocity);
    }

    @Override
    public void setVelocity(double value, double feedforward) {
        getPIDController().setReference(value, ControlType.kVelocity, 0, feedforward);
    }

    @Override
    public void setToCoastMode() {
        setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void setToBrakeMode() {
        setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void setCurrentLimit(double amp) {
        setSmartCurrentLimit((int) amp);
    }

    @Override
    public void setCurrentLimit(double amp, double timeMs) {
        setCurrentLimit(amp);
    }

    @Override
    public double getEncoderPosition() {
        return getEncoder().getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return getEncoder().getVelocity() * 2048 / 600;
    }

    @Override
    public void setEncoderPosition(double position) {
        getEncoder().setPosition(position);
    }

    @Override
    public void resetEncoder() {
        setEncoderPosition(0);
    }

    @Override
    public void setP(double value) {
        getPIDController().setP(value);
    }

    @Override
    public void setI(double value) {
        getPIDController().setI(value);
    }

    @Override
    public void setD(double value) {
        getPIDController().setD(value);
    }

    @Override
    public void setF(double value) {
        getPIDController().setFF(value);
    }

}
