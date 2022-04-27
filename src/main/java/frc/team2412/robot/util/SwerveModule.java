package frc.team2412.robot.util;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    public class SwerveModuleConstants {

    }

    WPI_TalonFX driveMotor;
    WPI_TalonFX turnMotor;

    boolean absoluteEncoderEnabled;
    CANCoder absoluteEncoder;

    public SwerveModule(int driveMotorPort, int turnMotorPort, double turnOffset, String canbus) {
        new SwerveModule(driveMotorPort, turnMotorPort, false, -1, turnOffset, canbus);
    }

    public SwerveModule(int driveMotorPort, int turnMotorPort, int turnEncoderPort, double turnOffset, String canbus) {
        new SwerveModule(driveMotorPort, turnMotorPort, true, turnEncoderPort, turnOffset, canbus);
    }

    public SwerveModule(int driveMotorPort, int turnMotorPort, boolean absoluteEncoderEnabled, int turnEncoderPort,
            double turnOffset, String canbus) {
        this.driveMotor = new WPI_TalonFX(driveMotorPort, canbus);
        this.turnMotor = new WPI_TalonFX(turnMotorPort, canbus);
        this.absoluteEncoderEnabled = absoluteEncoderEnabled;
        this.absoluteEncoder = new CANCoder(turnEncoderPort, canbus);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMeter(), getAngle());
    }

    public double getDriveVelocityMeter() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public Rotation2d getAngle() {
        double degree = absoluteEncoderEnabled ? absoluteEncoder.getAbsolutePosition()
                : turnMotor.getSelectedSensorPosition();
        return new Rotation2d(Math.toRadians(degree));
    }

}
