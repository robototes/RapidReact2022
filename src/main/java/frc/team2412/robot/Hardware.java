package frc.team2412.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import frc.team2412.robot.util.Mk4Configuration;
import org.frcteam2910.common.robot.drivers.NavX;
import org.photonvision.PhotonCamera;

import static frc.team2412.robot.Hardware.HardwareConstants.*;
import static frc.team2412.robot.SubsystemContainer.SubsystemConstants.*;

public class Hardware {
    public static class HardwareConstants {
        //drive
        public static final Mk4Configuration FRONT_LEFT_CONFIG = new Mk4Configuration(Mk4SwerveModuleHelper.GearRatio.L3, 0, 0, 0, -Math.toRadians(0));
        public static final Mk4Configuration FRONT_RIGHT_CONFIG = new Mk4Configuration(Mk4SwerveModuleHelper.GearRatio.L3, 0, 0, 0, -Math.toRadians(0));
        public static final Mk4Configuration BACK_LEFT_CONFIG = new Mk4Configuration(Mk4SwerveModuleHelper.GearRatio.L3, 0, 0, 0, -Math.toRadians(0));
        public static final Mk4Configuration BACK_RIGHT_CONFIG = new Mk4Configuration(Mk4SwerveModuleHelper.GearRatio.L3, 0, 0, 0, -Math.toRadians(0));

        public static final SPI.Port GYRO_PORT = SPI.Port.kMXP;

        //cameras
        public static final String LIMELIGHT = "limelight", FRONT_CAM = "front";

        //shooter
        public static final int FLYWHEEL_1 = 0, FLYWHEEL_2 = 0, TURRET = 0, HOOD = 0;

        //intake
        public static final int INTAKE_1 = 0, INTAKE_2 = 0, INTAKE_UP = 0, INTAKE_DOWN = 0;

        //index
        public static final int INDEX = 0;

        //climb
        public static final int CLIMB_FIXED_1 = 0, CLIMB_FIXED_2 = 0, CLIMB_ANGLED_1 = 0, CLIMB_ANGLED_2 = 0, CLIMB_ANGLE_UP = 0, CLIMB_ANGLE_DOWN = 0;
    }

    //drive
    public SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    public NavX navX;

    //cameras
    public PhotonCamera limelight, frontCamera;

    //shooter
    public TalonFX flywheelMotor1, flywheelMotor2, turretMotor, hoodMotor;

    //intake
    public WPI_TalonFX intakeMotor1, intakeMotor2;
    public DoubleSolenoid intakeSolenoid;

    //climb
    public TalonFX climbFixed1, climbFixed2, climbAngled1, climbAngled2;
    public DoubleSolenoid climbAngle;

    //index
    public TalonFX indexMotor;

    public Hardware() {
        if (DRIVE_ENABLED) {
            frontLeftModule = FRONT_LEFT_CONFIG.falcons();
            frontRightModule = FRONT_RIGHT_CONFIG.falcons();
            backLeftModule = BACK_LEFT_CONFIG.falcons();
            backRightModule = BACK_RIGHT_CONFIG.falcons();
            navX = new NavX(GYRO_PORT);
        }
        if (CLIMB_ENABLED) {
            climbFixed1 = new TalonFX(CLIMB_FIXED_1);
            climbFixed2 = new TalonFX(CLIMB_FIXED_2);
            climbAngled1 = new TalonFX(CLIMB_ANGLED_1);
            climbAngled2 = new TalonFX(CLIMB_ANGLED_2);
            climbAngle = new DoubleSolenoid(PneumaticsModuleType.REVPH, CLIMB_ANGLE_UP, CLIMB_ANGLE_DOWN);
        }
        if (INTAKE_ENABLED) {
            intakeMotor1 = new WPI_TalonFX(INTAKE_1);
            intakeMotor2 = new WPI_TalonFX(INTAKE_2);
            intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, INTAKE_UP, INTAKE_DOWN);
        }
        if (INDEX_ENABLED) {
            indexMotor = new TalonFX(INDEX);
        }
        if (SHOOTER_ENABLED) {
            flywheelMotor1 = new TalonFX(FLYWHEEL_1);
            flywheelMotor2 = new TalonFX(FLYWHEEL_2);
            turretMotor = new TalonFX(TURRET);
            hoodMotor = new TalonFX(HOOD);
        }
        if (FRONT_VIS_ENABLED) {
            frontCamera = new PhotonCamera(FRONT_CAM);
        }
        if (GOAL_VIS_ENABLED) {
            limelight = new PhotonCamera(LIMELIGHT);
        }
    }
}
