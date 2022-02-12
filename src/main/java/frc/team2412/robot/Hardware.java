package frc.team2412.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import org.frcteam2910.common.robot.drivers.NavX;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
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

        // drive can ids are range 1-19
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

        public static final SPI.Port GYRO_PORT = SPI.Port.kMXP;

        // cameras
        public static final String LIMELIGHT = "limelight", FRONT_CAM = "front";

        // shooter can ids are range 20-29
        public static final int FLYWHEEL_1 = 0, FLYWHEEL_2 = 0, TURRET = 0, HOOD = 0;

        // intake can ids are range 30-39
        public static final int INTAKE_1 = 32, INTAKE_2 = 34, INTAKE_UP = 36, INTAKE_DOWN = 38;

        // index can ids are range 40-49
        public static final int INDEX_INGEST_MOTOR = 40, INDEX_FEEDER_MOTOR = 41, INDEX_INGEST_SENSOR = 4,
                INDEX_FEEDER_SENSOR = 5;

        // climb can ids are range 50-59
        public static final int CLIMB_DYNAMIC = 0, CLIMB_FIXED = 0, CLIMB_ANGLE_UP = 0, CLIMB_ANGLE_DOWN = 0;

        // default address of TCA9548A
        public static final int I2C_MULTIPLEXER_ADDRESS = 0x70;
        public static final Port I2C_MULTIPLEXER_PORT = I2C.Port.kMXP;
        // which port the I2C device plugged in on the multiplexer
        public static final int LEFT_INTAKE_COLORSENSOR_PORT = 1, RIGHT_INTAKE_COLORSENSOR_PORT = 2,
                CENTER_INTAKE_COLORSENSOR_PORT = 3;
    }

    // drive
    public SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    public NavX navX;

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
    public MultiplexedColorSensor ingestIndexColorSensor, feederIndexColorSensor;

    public Hardware() {
        if (DRIVE_ENABLED) {
            frontLeftModule = FRONT_LEFT_CONFIG.falcons();
            frontRightModule = FRONT_RIGHT_CONFIG.falcons();
            backLeftModule = BACK_LEFT_CONFIG.falcons();
            backRightModule = BACK_RIGHT_CONFIG.falcons();
            navX = new NavX(GYRO_PORT);
        }
        if (CLIMB_ENABLED) {
            climbMotorDynamic = new WPI_TalonFX(CLIMB_DYNAMIC);
            climbMotorFixed = new WPI_TalonFX(CLIMB_FIXED);
            climbAngle = new DoubleSolenoid(PneumaticsModuleType.REVPH, CLIMB_ANGLE_UP, CLIMB_ANGLE_DOWN);
        }
        if (INTAKE_ENABLED) {
            intakeMotor1 = new WPI_TalonFX(INTAKE_1);
            intakeMotor2 = new WPI_TalonFX(INTAKE_2);
            intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, INTAKE_UP, INTAKE_DOWN);
            if (I2C_MUX_ENABLED) {
                this.leftIntakeColorSensor = new MultiplexedColorSensor(LEFT_INTAKE_COLORSENSOR_PORT);
                this.rightIntakeColorSensor = new MultiplexedColorSensor(RIGHT_INTAKE_COLORSENSOR_PORT);
                this.centerIntakeColorSensor = new MultiplexedColorSensor(CENTER_INTAKE_COLORSENSOR_PORT);
            }
        }
        if (INDEX_ENABLED) {
            ingestIndexMotor = new WPI_TalonFX(INDEX_INGEST_MOTOR);
            feederIndexMotor = new WPI_TalonFX(INDEX_FEEDER_MOTOR);
            ingestIndexColorSensor = new MultiplexedColorSensor(INDEX_INGEST_SENSOR);
            feederIndexColorSensor = new MultiplexedColorSensor(INDEX_FEEDER_SENSOR);
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
        if (GOAL_VIS_ENABLED) {
            limelight = new PhotonCamera(LIMELIGHT);
        }
    }
}
