package frc.team2412.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private static final double BATTERY_VOLTAGE = 12.8;
    private static final int ENCODERS_TICKS_PER_REVOLUTION = 2048;
    private static final double WHEEL_DIAMETER_METER = 0.10033;

    private static final double DRIVE_MOTOR_GEARING = 1 / 6.75;
    private static final boolean DRIVE_MOTOR_INVERTED = true;
    private static final double TURN_MOTOR_GEARING = 1 / 12.8;
    private static final boolean TURN_MOTOR_INVERTED = true;
    private static final double TURN_MOTOR_P = 0.2;
    private static final double TURN_MOTOR_D = 0.1;

    public WPI_TalonFX driveMotor;
    public WPI_TalonFX turnMotor;

    public SwerveModule(int driveMotorPort, int turnMotorPort, String canbus) {
        driveMotor = new WPI_TalonFX(driveMotorPort, canbus);
        turnMotor = new WPI_TalonFX(turnMotorPort, canbus);

        driveMotor.setInverted(DRIVE_MOTOR_INVERTED);
        driveMotor.configVoltageCompSaturation(BATTERY_VOLTAGE);
        driveMotor.enableVoltageCompensation(true);

        turnMotor.setInverted(TURN_MOTOR_INVERTED);
        turnMotor.configVoltageCompSaturation(BATTERY_VOLTAGE);
        turnMotor.enableVoltageCompensation(true);
        turnMotor.config_kP(0, TURN_MOTOR_P);
        turnMotor.config_kD(0, TURN_MOTOR_D);
    }

    // Speed gets normazlied to percent even though swerve module state is in velocity m
    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getAngle());

        driveMotor.setVoltage(state.speedMetersPerSecond * BATTERY_VOLTAGE);
        turnMotor.set(ControlMode.Position, state.angle.getDegrees());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMeter(), getAngle());
    }

    // conversion of velocity to enc per S to rotation per second
    public double getDriveVelocityMeter() {
        return driveMotor.getSelectedSensorVelocity() * 10 / ENCODERS_TICKS_PER_REVOLUTION * DRIVE_MOTOR_GEARING
                * WHEEL_DIAMETER_METER * Math.PI * 2;
    }

    public Rotation2d getAngle() {
        return new Rotation2d(turnMotor.getSelectedSensorPosition() / ENCODERS_TICKS_PER_REVOLUTION * TURN_MOTOR_GEARING
                * Math.PI * 2);
    }

    public void resetEncoder() {
        turnMotor.setSelectedSensorPosition(0);
        driveMotor.setSelectedSensorPosition(0);
    }

}
