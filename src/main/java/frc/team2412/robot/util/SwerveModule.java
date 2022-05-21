package frc.team2412.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private static final int ENCODERS_TICKS_PER_REVOLUTION = 2048;

    private static final double WHEEL_DIAMETER_METER = 0.10033;
    private static final double DRIVE_MOTOR_GEARING = 6.75;
    private static final double DRIVE_ENCODER_TICKS_TO_VELOCITY = 10 / DRIVE_MOTOR_GEARING
            / ENCODERS_TICKS_PER_REVOLUTION * Math.PI * WHEEL_DIAMETER_METER;
    private static final boolean DRIVE_MOTOR_INVERTED = true;

    private static final double DRIVE_MOTOR_P = 1;
    private static final double DRIVE_MOTOR_D = 0.1;

    private static final double TURN_MOTOR_GEARING = 12.8;
    private static final boolean TURN_MOTOR_INVERTED = true;
    private static final double TURN_ENCODER_TICKS_TO_DEGREE = 1 / TURN_MOTOR_GEARING / ENCODERS_TICKS_PER_REVOLUTION
            * 360;

    private static final double TURN_MOTOR_P = 0.2;
    private static final double TURN_MOTOR_D = 0.1;

    public WPI_TalonFX driveMotor;
    public WPI_TalonFX turnMotor;

    public SwerveModule(int driveMotorPort, int turnMotorPort, String canbus) {
        driveMotor = new WPI_TalonFX(driveMotorPort, canbus);
        turnMotor = new WPI_TalonFX(turnMotorPort, canbus);

        driveMotor.setInverted(DRIVE_MOTOR_INVERTED);
        driveMotor.config_kP(0, DRIVE_MOTOR_P);
        driveMotor.config_kD(0, DRIVE_MOTOR_D);

        turnMotor.setInverted(TURN_MOTOR_INVERTED);
        turnMotor.config_kP(0, TURN_MOTOR_P);
        turnMotor.config_kD(0, TURN_MOTOR_D);
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getAngle());
        driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond / DRIVE_ENCODER_TICKS_TO_VELOCITY);
        turnMotor.set(ControlMode.Position, state.angle.getDegrees() / TURN_ENCODER_TICKS_TO_DEGREE);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMeter(), getAngle());
    }

    public double getDriveVelocityMeter() {
        return driveMotor.getSelectedSensorVelocity() * DRIVE_ENCODER_TICKS_TO_VELOCITY;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(turnMotor.getSelectedSensorPosition() / TURN_ENCODER_TICKS_TO_DEGREE);
    }

    public void resetEncoder() {
        turnMotor.setSelectedSensorPosition(0);
        driveMotor.setSelectedSensorPosition(0);
    }

}
