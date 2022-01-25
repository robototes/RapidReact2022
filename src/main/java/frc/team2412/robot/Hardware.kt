package frc.team2412.robot

import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper
import com.swervedrivespecialties.swervelib.SwerveModule
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.SPI
import frc.team2412.robot.SubsystemContainer.SubsystemConstants.*
import frc.team2412.robot.util.Mk4Configuration
import org.frcteam2910.common.robot.drivers.NavX
import org.photonvision.PhotonCamera

public class Hardware {
    public companion object HardwareConstants {
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

        // Shooter
        public const val FLYWHEEL_1 = 0
        public const val FLYWHEEL_2 = 0
        public const val TURRET = 0
        public const val HOOD = 0

        // Intake
        public const val INTAKE_1 = 0
        public const val INTAKE_2 = 0
        public const val INTAKE_UP = 0
        public const val INTAKE_DOWN = 0

        // Index
        public const val INDEX = 0

        // Climb
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
    public lateinit var flywheelMotor1: TalonFX
    public lateinit var flywheelMotor2: TalonFX
    public lateinit var turretMotor: TalonFX
    public lateinit var hoodMotor: TalonFX

    // Intake
    public lateinit var intakeMotor1: TalonFX
    public lateinit var intakeMotor2: TalonFX
    public lateinit var intakeSolenoid: DoubleSolenoid

    // Climb
    public lateinit var climbFixed1: TalonFX
    public lateinit var climbFixed2: TalonFX
    public lateinit var climbAngled1: TalonFX
    public lateinit var climbAngled2: TalonFX
    public lateinit var climbAngle: DoubleSolenoid

    // Index
    public lateinit var indexMotor: TalonFX

    init {
        if (DRIVE_ENABLED) {
            frontLeftModule = FRONT_LEFT_CONFIG.falcons()
            frontRightModule = FRONT_RIGHT_CONFIG.falcons()
            backLeftModule = BACK_LEFT_CONFIG.falcons()
            backRightModule = BACK_RIGHT_CONFIG.falcons()
            navX = NavX(GYRO_PORT)
        }
        if (CLIMB_ENABLED) {
            climbFixed1 = TalonFX(CLIMB_FIXED_1)
            climbFixed2 = TalonFX(CLIMB_FIXED_2)
            climbAngled1 = TalonFX(CLIMB_ANGLED_1)
            climbAngled2 = TalonFX(CLIMB_ANGLED_2)
            climbAngle =
                    DoubleSolenoid(PneumaticsModuleType.REVPH, CLIMB_ANGLE_UP, CLIMB_ANGLE_DOWN)
        }
        if (INTAKE_ENABLED) {
            intakeMotor1 = TalonFX(INTAKE_1)
            intakeMotor2 = TalonFX(INTAKE_2)
            intakeSolenoid = DoubleSolenoid(PneumaticsModuleType.REVPH, INTAKE_UP, INTAKE_DOWN)
        }
        if (INDEX_ENABLED) {
            indexMotor = TalonFX(INDEX)
        }
        if (SHOOTER_ENABLED) {
            flywheelMotor1 = TalonFX(FLYWHEEL_1)
            flywheelMotor2 = TalonFX(FLYWHEEL_2)
            turretMotor = TalonFX(TURRET)
            hoodMotor = TalonFX(HOOD)
        }
        if (SHOOTER_ENABLED) {
            flywheelMotor1 = TalonFX(FLYWHEEL_1)
            flywheelMotor2 = TalonFX(FLYWHEEL_2)
            turretMotor = TalonFX(TURRET)
            hoodMotor = TalonFX(HOOD)
        }
        if (FRONT_VIS_ENABLED) {
            frontCamera = PhotonCamera(FRONT_CAM)
        }
        if (GOAL_VIS_ENABLED) {
            limelight = PhotonCamera(LIMELIGHT)
        }
    }
}
