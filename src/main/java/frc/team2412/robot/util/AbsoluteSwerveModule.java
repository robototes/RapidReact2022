package frc.team2412.robot.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class AbsoluteSwerveModule extends SwerveModule {

    private CANCoder absoluteEncoder;
    private final int turnEncoderPort;
    private final double turnOffset;
    private final String canbus;

    public AbsoluteSwerveModule(int driveMotorPort, int turnMotorPort, int turnEncoderPort, double turnOffset,
            String canbus) {
        super(driveMotorPort, turnMotorPort, canbus);
        this.turnEncoderPort = turnEncoderPort;
        this.turnOffset = turnOffset;
        this.canbus = canbus;
        configAbsoluteEncoder();

    }

    private void configAbsoluteEncoder() {
        absoluteEncoder = new CANCoder(turnEncoderPort, canbus);
        absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        absoluteEncoder.configMagnetOffset(turnOffset, 500);
        this.turnMotor.setSelectedSensorPosition(absoluteEncoder.getAbsolutePosition());
    }

    private int absoluteEncoderInitializationFailCount = 0;
    private final int absoluteEncoderInitializationFailThreshold = 10;

    @Override
    public void setState(SwerveModuleState state) {
        if (absoluteEncoder.setPositionToAbsolute().value != 0) {
            absoluteEncoderInitializationFailCount++;
        }
        if (absoluteEncoderInitializationFailCount > absoluteEncoderInitializationFailThreshold) {
            absoluteEncoderInitializationFailCount = 0;
            configAbsoluteEncoder();
        }
        super.setState(state);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
    }

    @Override
    public void resetEncoder() {
        super.resetEncoder();
        absoluteEncoder.setPosition(0);
    }

}
