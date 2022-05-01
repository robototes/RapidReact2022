package frc.team2412.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private static final int ENCODERS_TICKS_PER_REVOLUTION = 2048;
    private static final double WHEEL_DIAMTER_METER = 0.10033;

    private static final double DRIVE_MOTOR_GEARING = 1 / 6.75;
    private static final boolean DRIVE_MOTOR_INVERTED = true;
    private static final double TURN_MOTOR_GEARING = 1 / 12.8;
    private static final boolean TURN_MOTOR_INVERTED = true;

    WPI_TalonFX driveMotor;
    WPI_TalonFX turnMotor;

    public SwerveModule(int driveMotorPort, int turnMotorPort, String canbus) {
        driveMotor = new WPI_TalonFX(driveMotorPort, canbus);
        turnMotor = new WPI_TalonFX(turnMotorPort, canbus);
    }

    private void configMotors(){
        driveMotor.setInverted(DRIVE_MOTOR_INVERTED);
        driveMotor.configVoltageCompSaturation(12.8);
        driveMotor.enableVoltageCompensation(true);

        turnMotor.setInverted(TURN_MOTOR_INVERTED);
        turnMotor.configVoltageCompSaturation(12.8);
        turnMotor.enableVoltageCompensation(true);
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getAngle());
        driveMotor.set(state.speedMetersPerSecond);
        turnMotor.set(ControlMode.Position, state.angle.getDegrees());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMeter(), getAngle());
    }

    public double getDriveVelocityMeter() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public Rotation2d getAngle() {
        return new Rotation2d(Math.toRadians(turnMotor.getSelectedSensorPosition()));
    }

    public void resetEncoder() {
        turnMotor.setSelectedSensorPosition(0);
        driveMotor.setSelectedSensorPosition(0);
    }

}
