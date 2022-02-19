package frc.team2412.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.*;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.drivers.Pigeon;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import frc.team2412.robot.util.Mk4Configuration;
import frc.team2412.robot.util.MultiplexedColorSensor;

import static frc.team2412.robot.Hardware.HardwareConstants.*;
import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

public class Hardware {
    public static class HardwareConstants {

        // Color Sensor V3 Constants
        public static final Color BLUE_CARGO_COLOR = new Color(0.0, 0.4, 0.7019607844);
        public static final Color RED_CARGO_COLOR = new Color(0.9294117648, 0.1098039216, 0.1411764706);
        public static final double confidenceThreshold = 0.7;

        // drive can ids are range 1-19 (1 is taken by power distribution module)
        public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 1, DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 4,
                DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 7, DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 10;
        public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 2, DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 5,
                DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 8, DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 11;
        public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = -1, DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = -1,
                DRIVETRAIN_BACK_LEFT_ENCODER_PORT = -1, DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = -1;

        // TODO set encoder offset values
        public static final Mk4Configuration FRONT_LEFT_CONFIG = new Mk4Configuration(
                Mk4SwerveModuleHelper.GearRatio.L1,
                DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
                DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
                -Math.toRadians(0));
        public static final Mk4Configuration FRONT_RIGHT_CONFIG = new Mk4Configuration(
                Mk4SwerveModuleHelper.GearRatio.L1,
                DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
                DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
                -Math.toRadians(0));
        public static final Mk4Configuration BACK_LEFT_CONFIG = new Mk4Configuration(
                Mk4SwerveModuleHelper.GearRatio.L1,
                DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
                DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
                DRIVETRAIN_BACK_LEFT_ENCODER_PORT,
                -Math.toRadians(0));
        public static final Mk4Configuration BACK_RIGHT_CONFIG = new Mk4Configuration(
                Mk4SwerveModuleHelper.GearRatio.L1,
                DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
                DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
                DRIVETRAIN_BACK_RIGHT_ENCODER_PORT,
                -Math.toRadians(0));

        public static final double MODULE_MAX_RPM = 6000.0;
        public static final double MODULE_MAX_VELOCITY_METERS_PER_SEC = FRONT_LEFT_CONFIG.getRatio().getConfiguration()
                .getWheelDiameter() * Math.PI *
                FRONT_LEFT_CONFIG.getRatio().getConfiguration().getDriveReduction() * MODULE_MAX_RPM / 60.0;

        public static final int GYRO_PORT = 62;

        // cameras
        public static final String LIMELIGHT = "limelight", FRONT_CAM = "front";

        // shooter can ids are range 20-29
        public static final int FLYWHEEL_1 = 20, FLYWHEEL_2 = 21, TURRET = 22, HOOD = 23;

        // intake can ids are range 30-39
        public static final int INTAKE_INNER_MOTOR = 30, INTAKE_OUTER_MOTOR = 31, INTAKE_SOLENOID_UP = 14,
                INTAKE_SOLENOID_DOWN = 15;

        // index can ids are range 40-49
        public static final int INDEX_INGEST_MOTOR = 40, INDEX_FEEDER_MOTOR = 41, INDEX_INGEST_SENSOR = 4,
                INDEX_FEEDER_SENSOR = 5;

        // climb can ids are range 50-59
        public static final int CLIMB_DYNAMIC_MOTOR = 50, CLIMB_FIXED_MOTOR = 51, CLIMB_ANGLE_UP_SOLENOID = 7,
                CLIMB_ANGLE_DOWN_SOLENOID = 8;

        // default address of TCA9548A
        public static final int I2C_MULTIPLEXER_ADDRESS = 0x70;
        public static final Port I2C_MULTIPLEXER_PORT = I2C.Port.kMXP;
        // which port the I2C device plugged in on the multiplexer
        public static final int LEFT_INTAKE_COLORSENSOR_PORT = 1, RIGHT_INTAKE_COLORSENSOR_PORT = 2,
                CENTER_INTAKE_COLORSENSOR_PORT = 3;
    }

    // drive
    public SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    public Pigeon pidgeon;

    // cameras
    public PhotonCamera limelight, frontCamera;

    // shooter
    public WPI_TalonFX flywheelMotor1, flywheelMotor2, turretMotor;
    public CANSparkMax hoodMotor;

    // intake
    public WPI_TalonFX intakeMotor1, intakeMotor2;
    public DoubleSolenoid intakeSolenoid;
    public MultiplexedColorSensor leftIntakeColorSensor;
    public MultiplexedColorSensor rightIntakeColorSensor;
    public MultiplexedColorSensor centerIntakeColorSensor;

    // climb
    public WPI_TalonFX climbMotorFixed, climbMotorDynamic;

    public DoubleSolenoid climbAngle;

    // index
    public WPI_TalonFX ingestIndexMotor, feederIndexMotor;
    DigitalInput ingestProximity;
    DigitalInput feederProximity;

    public Hardware() {
        if (DRIVE_ENABLED) {
            frontLeftModule = FRONT_LEFT_CONFIG.falcons();
            frontRightModule = FRONT_RIGHT_CONFIG.falcons();
            backLeftModule = BACK_LEFT_CONFIG.falcons();
            backRightModule = BACK_RIGHT_CONFIG.falcons();
            pidgeon = new Pigeon(GYRO_PORT);
        }
        if (CLIMB_ENABLED) {
            climbMotorDynamic = new WPI_TalonFX(CLIMB_DYNAMIC_MOTOR);
            climbMotorFixed = new WPI_TalonFX(CLIMB_FIXED_MOTOR);
            climbAngle = new DoubleSolenoid(PneumaticsModuleType.REVPH, CLIMB_ANGLE_UP_SOLENOID,
                    CLIMB_ANGLE_DOWN_SOLENOID);
        }
        if (INTAKE_ENABLED) {
            intakeMotor1 = new WPI_TalonFX(INTAKE_INNER_MOTOR);
            intakeMotor2 = new WPI_TalonFX(INTAKE_OUTER_MOTOR);
            intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, INTAKE_SOLENOID_UP, INTAKE_SOLENOID_DOWN);
            if (I2C_MUX_ENABLED) {
                this.leftIntakeColorSensor = new MultiplexedColorSensor(LEFT_INTAKE_COLORSENSOR_PORT);
                this.rightIntakeColorSensor = new MultiplexedColorSensor(RIGHT_INTAKE_COLORSENSOR_PORT);
                this.centerIntakeColorSensor = new MultiplexedColorSensor(CENTER_INTAKE_COLORSENSOR_PORT);
            }
        }
        if (INDEX_ENABLED) {
            ingestIndexMotor = new WPI_TalonFX(INDEX_INGEST_MOTOR);
            feederIndexMotor = new WPI_TalonFX(INDEX_FEEDER_MOTOR);
            ingestProximity = new DigitalInput(0);
            feederProximity = new DigitalInput(1);

        }
        if (SHOOTER_ENABLED) {
            flywheelMotor1 = new WPI_TalonFX(FLYWHEEL_1);
            flywheelMotor2 = new WPI_TalonFX(FLYWHEEL_2);
            turretMotor = new WPI_TalonFX(TURRET);
            hoodMotor = new CANSparkMax(HOOD, CANSparkMaxLowLevel.MotorType.kBrushless);
        }
        if (DRIVER_VIS_ENABLED) {
            frontCamera = new PhotonCamera(FRONT_CAM);
        }
        if (SHOOTER_VISION_ENABLED) {
            limelight = new PhotonCamera(LIMELIGHT);
        }
    }
}
