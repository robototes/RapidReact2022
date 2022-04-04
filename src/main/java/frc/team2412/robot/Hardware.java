package frc.team2412.robot;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;

import frc.team2412.robot.sim.TalonFXSimProfile.TalonFXConstants;
import frc.team2412.robot.util.Mk4Configuration;

public class Hardware {
    // drive can ids are range 1-19 (1 is taken by power distribution module)
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 1, DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 4,
            DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 7, DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 10;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 2, DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 5,
            DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 8, DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 11;
    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 3, DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 6,
            DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 9, DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 12;
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(67.852);
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(221.924);
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(214.980);
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(168.398);
    public static final String DRIVETRAIN_INTAKE_CAN_BUS_NAME = "DrivebaseIntake";

    // Changes swerve modules & disables subsystems missing from the swerve test bot
    private static final Mk4SwerveModuleHelper.GearRatio GEAR_RATIO;

    static {
        GEAR_RATIO = Robot.getInstance().isCompetition()
                ? Mk4SwerveModuleHelper.GearRatio.L2
                : Mk4SwerveModuleHelper.GearRatio.L1;
    }

    public static final Mk4Configuration FRONT_LEFT_CONFIG = new Mk4Configuration(
            GEAR_RATIO,
            DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
            DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
            DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
            DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET,
            DRIVETRAIN_INTAKE_CAN_BUS_NAME);
    public static final Mk4Configuration FRONT_RIGHT_CONFIG = new Mk4Configuration(
            GEAR_RATIO,
            DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
            DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
            DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
            DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET,
            DRIVETRAIN_INTAKE_CAN_BUS_NAME);
    public static final Mk4Configuration BACK_LEFT_CONFIG = new Mk4Configuration(
            GEAR_RATIO,
            DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
            DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
            DRIVETRAIN_BACK_LEFT_ENCODER_PORT,
            DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET,
            DRIVETRAIN_INTAKE_CAN_BUS_NAME);
    public static final Mk4Configuration BACK_RIGHT_CONFIG = new Mk4Configuration(
            GEAR_RATIO,
            DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
            DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
            DRIVETRAIN_BACK_RIGHT_ENCODER_PORT,
            DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET,
            DRIVETRAIN_INTAKE_CAN_BUS_NAME);

    public static final double MODULE_MAX_RPM = 6000.0;
    public static final double MODULE_MAX_VELOCITY_METERS_PER_SEC = FRONT_LEFT_CONFIG.getRatio()
            .getConfiguration()
            .getWheelDiameter() * Math.PI *
            FRONT_LEFT_CONFIG.getRatio().getConfiguration().getDriveReduction() * MODULE_MAX_RPM
            / 60.0;

    public static final int GYRO_PORT = 62;

    // cameras
    public static final String LIMELIGHT = "limelight", FRONT_CAM = "front";

    // shooter can ids are range 20-29
    public static final int FLYWHEEL_1 = 20, FLYWHEEL_2 = 21, TURRET = 22, HOOD = 23;

    // intake can ids are range 30-39
    public static final int INTAKE_MOTOR_OUTER = 30, INTAKE_MOTOR_INNER = 31, INTAKE_SOLENOID_UP = 1,
            INTAKE_SOLENOID_DOWN = 0;

    // index can ids are range 40-49
    public static final int INDEX_INGEST_MOTOR = 40, INDEX_FEEDER_MOTOR = 41, INGEST_PROXIMITY = 1,
            LEFT_FEEDER_PROXIMITY = 0, RIGHT_FEEDER_PROXIMITY = 2;

    // climb can ids are range 50-59
    public static final int CLIMB_FIXED_MOTOR = 51;
    public static final int CLIMB_LIMIT_SWITCH = 9; // to be determined - digital I/O pins are 0-9
    public static final int POST_CLIMB_SOLENOID = 2; // TODO

    // Other hardware
    public static final int PDP_ID = 1; // needs to be verified on the bot (Can be found in REV)
    public static final int PNEUMATIC_HUB = 60;

    // Simulation stuff
    // TODO Find more accurate values
    public static final double SIM_FULL_VELOCITY = 6000 * TalonFXConstants.RPM_TO_VELOCITY;
}
