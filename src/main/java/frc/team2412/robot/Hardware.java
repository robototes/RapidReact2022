package frc.team2412.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;

import org.frcteam2910.common.drivers.Gyroscope;

import frc.team2412.robot.util.Mk4Configuration;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.drivers.Pigeon;

import static frc.team2412.robot.Hardware.HardwareConstants.*;
import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

public class Hardware {
    public static class HardwareConstants {

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

        // Changes swerve modules & disables subsystems missing from the swerve test bot
        public static final boolean COMPETITION_CONFIG = true;
        private static final Mk4SwerveModuleHelper.GearRatio GEAR_RATIO = COMPETITION_CONFIG
                ? Mk4SwerveModuleHelper.GearRatio.L2
                : Mk4SwerveModuleHelper.GearRatio.L1;

        public static final Mk4Configuration FRONT_LEFT_CONFIG = new Mk4Configuration(
                GEAR_RATIO,
                DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
                DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
                DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET);
        public static final Mk4Configuration FRONT_RIGHT_CONFIG = new Mk4Configuration(
                GEAR_RATIO,
                DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
                DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
                DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET);
        public static final Mk4Configuration BACK_LEFT_CONFIG = new Mk4Configuration(
                GEAR_RATIO,
                DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
                DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
                DRIVETRAIN_BACK_LEFT_ENCODER_PORT,
                DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET);
        public static final Mk4Configuration BACK_RIGHT_CONFIG = new Mk4Configuration(
                GEAR_RATIO,
                DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
                DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
                DRIVETRAIN_BACK_RIGHT_ENCODER_PORT,
                DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET);

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
        public static final int INTAKE_MOTOR = 30, INTAKE_SOLENOID_UP = 14,
                INTAKE_SOLENOID_DOWN = 15;

        // index can ids are range 40-49
        public static final int INDEX_INGEST_MOTOR = 40, INDEX_FEEDER_MOTOR = 41, INGEST_RED = 0, INGEST_BLUE = 1,
                INGEST_PROXIMITY = 2, FEEDER_RED = 3, FEEDER_BLUE = 4, FEEDER_PROXIMITY = 5;

        // climb can ids are range 50-59
        public static final int CLIMB_DYNAMIC_MOTOR = 50, CLIMB_FIXED_MOTOR = 51, CLIMB_ANGLE_UP_SOLENOID = 7,
                CLIMB_ANGLE_DOWN_SOLENOID = 8;
    }

    // drive
    public SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    public Gyroscope gyro;

    // cameras
    public UsbCamera frontCamera;

    // shooter
    public WPI_TalonFX flywheelMotor1, flywheelMotor2, turretMotor;
    public CANSparkMax hoodMotor;

    // intake
    public WPI_TalonFX intakeMotor;
    public DoubleSolenoid intakeSolenoid;

    // climb
    public WPI_TalonFX climbMotorFixed, climbMotorDynamic;

    public DoubleSolenoid climbAngle;

    // index
    public WPI_TalonFX ingestIndexMotor, feederIndexMotor;
    public DigitalInput ingestProximity;
    public DigitalInput feederProximity;
    public DigitalInput ingestBlueColor;
    public DigitalInput ingestRedColor;
    public DigitalInput feederBlueColor;
    public DigitalInput feederRedColor;

    public Hardware() {
        boolean comp = Robot.getInstance().isCompetition();
        if (DRIVE_ENABLED) {
            frontLeftModule = FRONT_LEFT_CONFIG.create(comp);
            frontRightModule = FRONT_RIGHT_CONFIG.create(comp);
            backLeftModule = BACK_LEFT_CONFIG.create(comp);
            backRightModule = BACK_RIGHT_CONFIG.create(comp);
            gyro = comp ? new Pigeon(GYRO_PORT) : new NavX(SerialPort.Port.kMXP);
        }
        if (CLIMB_ENABLED) {
            climbMotorDynamic = new WPI_TalonFX(CLIMB_DYNAMIC_MOTOR);
            climbMotorFixed = new WPI_TalonFX(CLIMB_FIXED_MOTOR);
            climbAngle = new DoubleSolenoid(PneumaticsModuleType.REVPH, CLIMB_ANGLE_UP_SOLENOID,
                    CLIMB_ANGLE_DOWN_SOLENOID);
        }
        if (INTAKE_ENABLED) {
            intakeMotor = new WPI_TalonFX(INTAKE_MOTOR);
            intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, INTAKE_SOLENOID_UP,
                    INTAKE_SOLENOID_DOWN);
        }
        if (INDEX_ENABLED) {
            ingestIndexMotor = new WPI_TalonFX(INDEX_INGEST_MOTOR);
            feederIndexMotor = new WPI_TalonFX(INDEX_FEEDER_MOTOR);
            ingestProximity = new DigitalInput(INGEST_PROXIMITY);
            feederProximity = new DigitalInput(FEEDER_PROXIMITY);
            ingestBlueColor = new DigitalInput(INGEST_BLUE);
            ingestRedColor = new DigitalInput(INGEST_RED);
            feederBlueColor = new DigitalInput(FEEDER_BLUE);
            feederRedColor = new DigitalInput(FEEDER_RED);

        }
        if (SHOOTER_ENABLED) {
            flywheelMotor1 = new WPI_TalonFX(FLYWHEEL_1);
            flywheelMotor2 = new WPI_TalonFX(FLYWHEEL_2);
            turretMotor = new WPI_TalonFX(TURRET);
            hoodMotor = new CANSparkMax(HOOD, CANSparkMaxLowLevel.MotorType.kBrushless);
        }
        if (DRIVER_VIS_ENABLED) {
            CameraServer.addCamera(frontCamera);
            CameraServer.startAutomaticCapture();
        }
        if (SHOOTER_VISION_ENABLED) {
        }
    }
}
