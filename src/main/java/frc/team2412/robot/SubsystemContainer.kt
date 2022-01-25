package frc.team2412.robot

import frc.team2412.robot.subsystems.*

public class SubsystemContainer(h: Hardware) {
    public companion object SubsystemConstants {
        public const val CLIMB_ENABLED = false
        public const val DRIVE_ENABLED = false
        public const val FRONT_VIS_ENABLED = false
        public const val GOAL_VIS_ENABLED = false
        public const val INDEX_ENABLED = false
        public const val INTAKE_ENABLED = false
        public const val SHOOTER_ENABLED = false
    }

    public val hardware = h

    public lateinit var climbSubsystem: ClimbSubsystem
    public lateinit var drivebaseSubsystem: DrivebaseSubsystem
    public lateinit var frontVisionSubsystem: FrontVisionSubsystem
    public lateinit var goalVisionSubsystem: GoalVisionSubsystem
    public lateinit var indexSubsystem: IndexSubsystem
    public lateinit var intakeSubsystem: IntakeSubsystem
    public lateinit var shooterSubsystem: ShooterSubsystem

    init {
        if (CLIMB_ENABLED)
                climbSubsystem =
                        ClimbSubsystem(
                                hardware.climbFixed1,
                                hardware.climbFixed2,
                                hardware.climbAngled1,
                                hardware.climbAngled2,
                                hardware.climbAngle
                        )

        if (DRIVE_ENABLED)
                drivebaseSubsystem =
                        DrivebaseSubsystem(
                                hardware.frontLeftModule,
                                hardware.frontRightModule,
                                hardware.backLeftModule,
                                hardware.backRightModule,
                                hardware.navX
                        )

        if (FRONT_VIS_ENABLED) frontVisionSubsystem = FrontVisionSubsystem(hardware.frontCamera)

        if (GOAL_VIS_ENABLED) goalVisionSubsystem = GoalVisionSubsystem(hardware.limelight)

        if (INDEX_ENABLED) indexSubsystem = IndexSubsystem(hardware.indexMotor)

        if (INTAKE_ENABLED)
                intakeSubsystem =
                        IntakeSubsystem(
                                hardware.intakeMotor1,
                                hardware.intakeMotor2,
                                hardware.intakeSolenoid1,
                                hardware.intakeSolenoid2,
                                hardware.intakeColorSensor
                        )

        if (SHOOTER_ENABLED)
                shooterSubsystem =
                        ShooterSubsystem(
                                hardware.flywheelMotor1,
                                hardware.flywheelMotor2,
                                hardware.turretMotor,
                                hardware.hoodMotor
                        )
    }
}
