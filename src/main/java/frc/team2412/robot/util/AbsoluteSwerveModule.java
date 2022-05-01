package frc.team2412.robot.util;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class AbsoluteSwerveModule extends SwerveModule {

    CANCoder absoluteEncoder;

    public AbsoluteSwerveModule(int driveMotorPort, int turnMotorPort, int turnEncoderPort, double turnOffset,
            String canbus) {
        super(driveMotorPort, turnMotorPort, canbus);

        if (turnEncoderPort != -1) {
            configAbsoluteEncoder(turnEncoderPort, turnOffset, canbus);
        }

    }

    private void configAbsoluteEncoder(int turnEncoderPort, double turnOffset, String canbus) {

        absoluteEncoder = new CANCoder(turnEncoderPort, canbus);
        ErrorCode encoderCreatedCheck = absoluteEncoder.configMagnetOffset(turnOffset, 500);

        // I feel like this will break but 🤷‍♂️
        if (encoderCreatedCheck.value != 0) {
            configAbsoluteEncoder(turnEncoderPort, turnOffset, canbus);
        }

        this.turnMotor.setSelectedSensorPosition(absoluteEncoder.getAbsolutePosition());
    }

    @Override
    public Rotation2d getAngle() {
        return new Rotation2d(Math.toRadians(absoluteEncoder.getAbsolutePosition()));
    }

    public void resetEncoder() {
        super.resetEncoder();
        absoluteEncoder.setPosition(0);
    }

}
