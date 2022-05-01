package frc.team2412.robot.util;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.ctre.CtreUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    public static class SwerveModuleConstants {

    }

    WPI_TalonFX driveMotor;
    WPI_TalonFX turnMotor;
    CANCoder absoluteEncoder;

    public SwerveModule(int driveMotorPort, int turnMotorPort, int turnEncoderPort, double turnOffset, String canbus) {
        driveMotor = new WPI_TalonFX(driveMotorPort, canbus);
        turnMotor = new WPI_TalonFX(turnMotorPort, canbus);

        if (turnEncoderPort != -1) {
            configAbsoluteEncoder(turnEncoderPort, turnOffset, canbus);
        }

    }

    private void configAbsoluteEncoder(int turnEncoderPort, double turnOffset, String canbus){

        absoluteEncoder = new CANCoder(turnEncoderPort, canbus);
        ErrorCode encoderCreatedCheck = absoluteEncoder.configMagnetOffset(turnOffset, 100);

        // I feel like this will break but 🤷‍♂️
        if(encoderCreatedCheck.value != 0){
            configAbsoluteEncoder(turnEncoderPort, turnOffset, canbus);
        }


        this.turnMotor.setSelectedSensorPosition(absoluteEncoder.getAbsolutePosition());
    }

    

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMeter(), getAngle());
    }

    public double getDriveVelocityMeter() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public Rotation2d getAngle() {
        double degree = absoluteEncoder != null ? absoluteEncoder.getAbsolutePosition()
                : turnMotor.getSelectedSensorPosition();
        return new Rotation2d(Math.toRadians(degree));
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getAngle());
        driveMotor.set(state.speedMetersPerSecond);
        turnMotor.set(ControlMode.Position, state.angle.getDegrees());
    }

    public void resetEncoder() {
        if (absoluteEncoder != null) {
            absoluteEncoder.setPosition(0);
        }
        turnMotor.setSelectedSensorPosition(0);
        driveMotor.setSelectedSensorPosition(0);
    }

}
