package frc.team2412.robot.util.MotorStuff;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface Motor extends MotorController {

    public void setPosition(double value);

    public void setPosition(double value, double feedforward);

    public void setVelocity(double value);

    public void setVelocity(double value, double feedforward);

    public void setVoltage(double voltage);

    public void setToCoastMode();

    public void setToBrakeMode();

    public void setCurrentLimit(double amp);

    public void setCurrentLimit(double amp, double timeMs);

    public double getEncoderPosition();

    public double getEncoderVelocity();

    public void setEncoderPosition(double position);

    public void resetEncoder();

    public void setP(double value);

    public void setD(double value);

    public void setF(double value);

}
