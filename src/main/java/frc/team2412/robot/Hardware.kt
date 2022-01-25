package frc.team2412.robot

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.revrobotics.ColorSensorV3
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper
import com.swervedrivespecialties.swervelib.SwerveModule
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Solenoid
import frc.team2412.robot.Subsystems.SubsystemConstants
import frc.team2412.robot.util.Mk4Configuration
import org.frcteam2910.common.robot.drivers.NavX
import org.photonvision.PhotonCamera

public class Hardware {
    public companion object HardwareConstants {
        // Drive CAN ids are range 1-19
        public val FRONT_LEFT_CONFIG =
                Mk4Configuration(Mk4SwerveModuleHelper.GearRatio.L3, 0, 0, 0, -Math.toRadians(0.0))
        public val FRONT_RIGHT_CONFIG =
                Mk4Configuration(Mk4SwerveModuleHelper.GearRatio.L3, 0, 0, 0, -Math.toRadians(0.0))
        public val BACK_LEFT_CONFIG =
                Mk4Configuration(Mk4SwerveModuleHelper.GearRatio.L3, 0, 0, 0, -Math.toRadians(0.0))
        public val BACK_RIGHT_CONFIG =
                Mk4Configuration(Mk4SwerveModuleHelper.GearRatio.L3, 0, 0, 0, -Math.toRadians(0.0))

        public val GYRO_PORT = SPI.Port.kMXP

        // Cameras
        public const val LIMELIGHT = "limelight"
        public const val FRONT_CAM = "front"

        // Shooter CAN ids are range 20-29
        public const val FLYWHEEL_1 = 0
        public const val FLYWHEEL_2 = 0
        public const val TURRET = 0
        public const val HOOD = 0

        // Intake CAN ids are range 30-39
        public const val INTAKE_1 = 0
        public const val INTAKE_2 = 0
        public const val INTAKE_UP = 0
        public const val INTAKE_DOWN = 0

        // Index CAN ids are range 40-49
        public const val INDEX = 0

        // Climb CAN ids are range 50-59
        public const val CLIMB_FIXED_1 = 0
        public const val CLIMB_FIXED_2 = 0
        public const val CLIMB_ANGLED_1 = 0
        public const val CLIMB_ANGLED_2 = 0
        public const val CLIMB_ANGLE_UP = 0
        public const val CLIMB_ANGLE_DOWN = 0
    }

    // Drive
    public lateinit var frontLeftModule: SwerveModule
    public lateinit var frontRightModule: SwerveModule
    public lateinit var backLeftModule: SwerveModule
    public lateinit var backRightModule: SwerveModule
    public lateinit var navX: NavX

    // Cameras
    public lateinit var limelight: PhotonCamera
    public lateinit var frontCamera: PhotonCamera

    // Shooter
    public lateinit var flywheelMotor1: WPI_TalonFX
    public lateinit var flywheelMotor2: WPI_TalonFX
    public lateinit var turretMotor: WPI_TalonFX
    public lateinit var hoodMotor: WPI_TalonFX

    // Intake
    public lateinit var intakeMotor1: WPI_TalonFX
    public lateinit var intakeMotor2: WPI_TalonFX
    public lateinit var intakeSolenoid1: Solenoid
    public lateinit var intakeSolenoid2: Solenoid
    public lateinit var intakeColorSensor: ColorSensorV3

    // Climb
    public lateinit var climbFixed1: WPI_TalonFX
    public lateinit var climbFixed2: WPI_TalonFX
    public lateinit var climbAngle: DoubleSolenoid

    // Index
    public lateinit var indexMotor: WPI_TalonFX

    init {
        if (SubsystemConstants.DRIVE_ENABLED) {
            frontLeftModule = FRONT_LEFT_CONFIG.falcons()
            frontRightModule = FRONT_RIGHT_CONFIG.falcons()
            backLeftModule = BACK_LEFT_CONFIG.falcons()
            backRightModule = BACK_RIGHT_CONFIG.falcons()
            navX = NavX(GYRO_PORT)
        }
        if (SubsystemConstants.CLIMB_ENABLED) {
            climbFixed1 = WPI_TalonFX(CLIMB_FIXED_1)
            climbFixed2 = WPI_TalonFX(CLIMB_FIXED_2)
            climbAngle =
                    DoubleSolenoid(PneumaticsModuleType.REVPH, CLIMB_ANGLE_UP, CLIMB_ANGLE_DOWN)
        }
        if (SubsystemConstants.INTAKE_ENABLED) {
            intakeMotor1 = WPI_TalonFX(INTAKE_1)
            intakeMotor2 = WPI_TalonFX(INTAKE_2)
            intakeSolenoid1 = Solenoid(PneumaticsModuleType.REVPH, INTAKE_UP)
            intakeSolenoid2 = Solenoid(PneumaticsModuleType.REVPH, INTAKE_DOWN)
        }
        if (SubsystemConstants.INDEX_ENABLED) {
            indexMotor = WPI_TalonFX(INDEX)
        }
        if (SubsystemConstants.SHOOTER_ENABLED) {
            flywheelMotor1 = WPI_TalonFX(FLYWHEEL_1)
            flywheelMotor2 = WPI_TalonFX(FLYWHEEL_2)
            turretMotor = WPI_TalonFX(TURRET)
            hoodMotor = WPI_TalonFX(HOOD)
        }
        if (SubsystemConstants.DRIVER_VIS_ENABLED) {
            frontCamera = PhotonCamera(FRONT_CAM)
        }
        if (SubsystemConstants.GOAL_VIS_ENABLED) {
            limelight = PhotonCamera(LIMELIGHT)
        }
    }
}
