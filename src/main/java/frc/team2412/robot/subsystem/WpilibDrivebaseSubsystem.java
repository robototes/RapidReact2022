package frc.team2412.robot.subsystem;

import static frc.team2412.robot.Hardware.*;

import java.util.Arrays;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import org.frcteam2910.common.robot.drivers.NavX;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.util.SumedhsDatatouilleSwerveModule;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.SerialPort;

public class WpilibDrivebaseSubsystem extends SubsystemBase implements Loggable {

    private final SumedhsDatatouilleSwerveModule frontLeft = new SumedhsDatatouilleSwerveModule(DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
            DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    private final SumedhsDatatouilleSwerveModule frontRight = new SumedhsDatatouilleSwerveModule(DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
            DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    private final SumedhsDatatouilleSwerveModule backLeft = new SumedhsDatatouilleSwerveModule(DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
            DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    private final SumedhsDatatouilleSwerveModule backRight = new SumedhsDatatouilleSwerveModule(DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
            DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    private final static double TRACKWIDTH_METER = Units.inchesToMeters(20);

    private final Translation2d frontLeftLocation = new Translation2d(TRACKWIDTH_METER / 2, TRACKWIDTH_METER / 2);
    private final Translation2d frontRightLocation = new Translation2d(TRACKWIDTH_METER / 2, -TRACKWIDTH_METER / 2);
    private final Translation2d backLeftLocation = new Translation2d(-TRACKWIDTH_METER / 2, TRACKWIDTH_METER / 2);
    private final Translation2d backRightLocation = new Translation2d(-TRACKWIDTH_METER / 2, -TRACKWIDTH_METER / 2);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getGyroHeading());

    public final double maxVelocityMetersPerSecond = 2.4115;

    public WpilibDrivebaseSubsystem() {

    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        // xSpeed = 0.1;
        // ySpeed = 0.1;
        // rot = 0;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocityMetersPerSecond);
        setStates(swerveModuleStates);
    }

    public void setStates(SwerveModuleState[] swerveModuleStates) {
        System.out.println(Arrays.deepToString(swerveModuleStates));
        frontLeft.setState(swerveModuleStates[0]);
        frontRight.setState(swerveModuleStates[1]);
        backLeft.setState(swerveModuleStates[2]);
        backRight.setState(swerveModuleStates[3]);

        System.out.println(frontLeft.getAngle());
        
        System.out.println(backLeft.getAngle());
        
        System.out.println(frontRight.getAngle());
        
        System.out.println(backRight.getAngle());

    }

    public Rotation2d getGyroHeading() {
        return new Rotation2d(gyro.getYaw());
    }

    public void resetGyro() {
        gyro.getAngle();
    }

    public void updateOdometry() {
        odometry.update(
                getGyroHeading(),
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState());
    }

    public void setPose(Pose2d pose, Rotation2d gyroAngle) {
        odometry.resetPosition(pose, gyroAngle);
    }

    public void resetPose() {
        setPose(new Pose2d(), new Rotation2d());
    }

    @Log.ToString
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
}
