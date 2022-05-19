package frc.team2412.robot.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;

public class AbsoluteSwerveModule extends SwerveModule {

    private CANCoder absoluteEncoder;

    public AbsoluteSwerveModule(int driveMotorPort, int turnMotorPort, int turnEncoderPort, double turnOffset,
            String canbus) {
        super(driveMotorPort, turnMotorPort, canbus);
        configAbsoluteEncoder(turnEncoderPort, turnOffset, canbus);

    }

    private void configAbsoluteEncoder(int turnEncoderPort, double turnOffset, String canbus) {

        absoluteEncoder = new CANCoder(turnEncoderPort, canbus);
        absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        absoluteEncoder.configMagnetOffset(turnOffset, 500);
        this.turnMotor.setSelectedSensorPosition(absoluteEncoder.getAbsolutePosition());
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
    }

    public void resetEncoder() {
        super.resetEncoder();
        absoluteEncoder.setPosition(0);
    }

}
