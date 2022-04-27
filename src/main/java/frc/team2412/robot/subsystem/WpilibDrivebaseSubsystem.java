package frc.team2412.robot.subsystem;

import static frc.team2412.robot.Hardware.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.util.SwerveModule;

public class WpilibDrivebaseSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
            DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, DRIVETRAIN_FRONT_LEFT_ENCODER_PORT, DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET,
            DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    private final SwerveModule frontRight = new SwerveModule(DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
            DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
            DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET,
            DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    private final SwerveModule backLeft = new SwerveModule(DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
            DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, DRIVETRAIN_BACK_LEFT_ENCODER_PORT, DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET,
            DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    private final SwerveModule backRight = new SwerveModule(DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
            DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, DRIVETRAIN_BACK_RIGHT_ENCODER_PORT, DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET,
            DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    // copied from WPILIB
    private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    public WpilibDrivebaseSubsystem() {

    }

}
