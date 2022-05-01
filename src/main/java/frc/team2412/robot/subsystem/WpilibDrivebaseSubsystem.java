package frc.team2412.robot.subsystem;

import static frc.team2412.robot.Hardware.*;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.util.AbsoluteSwerveModule;

public class WpilibDrivebaseSubsystem extends SubsystemBase {

    private final AbsoluteSwerveModule frontLeft = new AbsoluteSwerveModule(DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
            DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, DRIVETRAIN_FRONT_LEFT_ENCODER_PORT, DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET,
            DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    private final AbsoluteSwerveModule frontRight = new AbsoluteSwerveModule(DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
            DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
            DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET,
            DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    private final AbsoluteSwerveModule backLeft = new AbsoluteSwerveModule(DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
            DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, DRIVETRAIN_BACK_LEFT_ENCODER_PORT, DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET,
            DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    private final AbsoluteSwerveModule backRight = new AbsoluteSwerveModule(DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
            DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, DRIVETRAIN_BACK_RIGHT_ENCODER_PORT, DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET,
            DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    private final static double TRACKWIDTH_METER = Units.inchesToMeters(20);

    // copied from WPILIB
    private final Translation2d frontLeftLocation = new Translation2d(TRACKWIDTH_METER / 2, TRACKWIDTH_METER / 2);
    private final Translation2d frontRightLocation = new Translation2d(TRACKWIDTH_METER / 2, -TRACKWIDTH_METER / 2);
    private final Translation2d backLeftLocation = new Translation2d(-TRACKWIDTH_METER / 2, TRACKWIDTH_METER / 2);
    private final Translation2d backRightLocation = new Translation2d(-TRACKWIDTH_METER / 2, -TRACKWIDTH_METER / 2);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final Pigeon2 gyro = new Pigeon2(GYRO_PORT, DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getGyroHeading());

    public WpilibDrivebaseSubsystem() {

    }

    @Override
    public void periodic() {
        odometry.update(getGyroHeading(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroHeading())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 1);

        frontLeft.setState(swerveModuleStates[0]);
        frontRight.setState(swerveModuleStates[1]);
        backLeft.setState(swerveModuleStates[2]);
        backRight.setState(swerveModuleStates[3]);
    }

    public Rotation2d getGyroHeading() {
        return new Rotation2d(gyro.getYaw());
    }

    public void resetGyro() {
        gyro.setYaw(0);
    }

}
