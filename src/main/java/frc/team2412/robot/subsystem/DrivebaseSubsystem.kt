package frc.team2412.robot.subsystem

import com.google.errorprone.annotations.concurrent.GuardedBy
import com.swervedrivespecialties.swervelib.SwerveModule
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.frcteam2910.common.control.CentripetalAccelerationConstraint
import org.frcteam2910.common.control.FeedforwardConstraint
import org.frcteam2910.common.control.HolonomicMotionProfiledTrajectoryFollower
import org.frcteam2910.common.control.MaxAccelerationConstraint
import org.frcteam2910.common.control.PidConstants
import org.frcteam2910.common.control.TrajectoryConstraint
import org.frcteam2910.common.drivers.Gyroscope
import org.frcteam2910.common.kinematics.ChassisVelocity
import org.frcteam2910.common.kinematics.SwerveKinematics
import org.frcteam2910.common.kinematics.SwerveOdometry
import org.frcteam2910.common.math.RigidTransform2
import org.frcteam2910.common.math.Rotation2
import org.frcteam2910.common.math.Vector2
import org.frcteam2910.common.robot.UpdateManager
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants
import org.frcteam2910.common.util.HolonomicDriveSignal
import org.frcteam2910.common.util.HolonomicFeedforward
import org.frcteam2910.common.util.InterpolatingDouble
import org.frcteam2910.common.util.InterpolatingTreeMap

public class DrivebaseSubsystem(
        fl: SwerveModule,
        fr: SwerveModule,
        bl: SwerveModule,
        br: SwerveModule,
        g: Gyroscope
) : SubsystemBase(), UpdateManager.Updatable {
    // TODO: find values as these are just copied from 2910
    public companion object DriveConstants {
        public const val TRACKWIDTH = 1.0
        public const val WHEELBASE = 1.0

        public val FEEDFORWARD_CONSTANTS =
                DrivetrainFeedforwardConstants(0.042746, 0.0032181, 0.30764)

        public val TRAJECTORY_CONSTRAINTS: Array<TrajectoryConstraint> =
                arrayOf(
                        FeedforwardConstraint(
                                11.0,
                                FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                                FEEDFORWARD_CONSTANTS.getAccelerationConstant(),
                                false
                        ),
                        MaxAccelerationConstraint(12.5 * 12.0),
                        CentripetalAccelerationConstraint(15 * 12.0)
                )

        public const val MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25
    }

    private val follower =
            HolonomicMotionProfiledTrajectoryFollower(
                    PidConstants(0.4, 0.0, 0.025),
                    PidConstants(5.0, 0.0, 0.0),
                    HolonomicFeedforward(FEEDFORWARD_CONSTANTS)
            )

    private val swerveKinematics =
            SwerveKinematics(
                    Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // front left
                    Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // front right
                    Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // back left
                    Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // back right
            )

    private val wpi_driveKinematics =
            SwerveDriveKinematics(
                    Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // front left
                    Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // front right
                    Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // back left
                    Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // back right
            )

    private val modules = arrayOf(fl, fr, bl, br)

    private val sensorLock = Object()
    @GuardedBy("sensorLock") private val gyroscope = g

    private val kinematicsLock = Object()
    @GuardedBy("kinematicsLock")
    private val swerveOdometry = SwerveOdometry(swerveKinematics, RigidTransform2.ZERO)
    @GuardedBy("kinematicsLock") private var pose = RigidTransform2.ZERO
    @GuardedBy("kinematicsLock")
    private val latencyCompensationMap =
            InterpolatingTreeMap<InterpolatingDouble, RigidTransform2>()
    @GuardedBy("kinematicsLock") private var velocity = Vector2.ZERO
    @GuardedBy("kinematicsLock") private var angularVelocity = 0.0

    private val stateLock = Object()
    @GuardedBy("stateLock") private lateinit var driveSignal: HolonomicDriveSignal

    // Logging
    private val tab = Shuffleboard.getTab("Drivetrain")
    private val odometryXEntry = tab.add("X", 0.0).withPosition(0, 0).withSize(1, 1).getEntry()
    private val odometryYEntry = tab.add("Y", 0.0).withPosition(0, 1).withSize(1, 1).getEntry()
    private val odometryAngleEntry =
            tab.add("Angle", 0.0).withPosition(0, 2).withSize(1, 1).getEntry()

    init {
        synchronized(sensorLock) { gyroscope.setInverted(false) }

        tab.addNumber(
                        "Trajectory X",
                        {
                            if (follower.getLastState() == null) {
                                0.0
                            } else {
                                follower.getLastState().getPathState().getPosition().x
                            }
                        }
                )
                .withPosition(1, 1)
                .withSize(1, 1)

        tab.addNumber(
                        "Trajectory Y",
                        {
                            if (follower.getLastState() == null) {
                                0.0
                            } else {
                                follower.getLastState().getPathState().getPosition().y
                            }
                        }
                )
                .withPosition(1, 1)
                .withSize(1, 1)

        tab.addNumber(
                "Rotation Voltage",
                {
                    synchronized(stateLock) {
                        driveSignal.getRotation() * RobotController.getBatteryVoltage()
                    }
                }
        )

        tab.addNumber("Average Velocity", this::getAverageAbsoluteVelocity)
    }

    public fun getPose(): RigidTransform2 {
        synchronized(kinematicsLock) {
            return pose
        }
    }

    public fun getVelocity(): Vector2 {
        synchronized(kinematicsLock) {
            return velocity
        }
    }

    public fun getAngularVelocity(): Double {
        synchronized(kinematicsLock) {
            return angularVelocity
        }
    }

    public fun drive(
            translationalVelocity: Vector2,
            rotationalVelocity: Double,
            isFieldOriented: Boolean
    ) {
        synchronized(stateLock) {
            driveSignal =
                    HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented)
        }
    }

    public fun resetPose(pose: RigidTransform2) {
        synchronized(kinematicsLock) {
            this.pose = pose
            swerveOdometry.resetPose(pose)
        }
    }

    public fun resetGyroAngle(angle: Rotation2) {
        synchronized(sensorLock) {
            gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle().rotateBy(angle.inverse()))
        }
    }

    public fun getAverageAbsoluteVelocity(): Double {
        var averageVelocity = 0.0
        for (module in modules) {
            averageVelocity += Math.abs(module.getDriveVelocity())
        }
        return averageVelocity / 4
    }

    private fun updateOdometry(time: Double, dt: Double) {
        val moduleVelocities = arrayOfNulls<Vector2>(modules.size)
        for (i in 0..modules.size) {
            var module = modules[i]

            moduleVelocities[i] =
                    Vector2.fromAngle(Rotation2.fromRadians(module.getSteerAngle()))
                            .scale(module.getDriveVelocity() * 39.37008)
        }

        lateinit var angle: Rotation2
        var angularVelocity = 0.0
        synchronized(sensorLock) {
            angle = gyroscope.getAngle()
            angularVelocity = gyroscope.getRate()
        }

        val velocity = swerveKinematics.toChassisVelocity(*moduleVelocities)

        synchronized(kinematicsLock) {
            this.pose = swerveOdometry.update(angle, dt, *moduleVelocities)
            if (latencyCompensationMap.size > MAX_LATENCY_COMPENSATION_MAP_ENTRIES) {
                latencyCompensationMap.remove(latencyCompensationMap.firstKey())
            }
            latencyCompensationMap.put(InterpolatingDouble(time), pose)
            this.velocity = velocity.getTranslationalVelocity()
            this.angularVelocity = angularVelocity
        }
    }

    private fun updateModules(driveSignal: HolonomicDriveSignal, dt: Double) {
        lateinit var chassisVelocity: ChassisVelocity
        if (driveSignal.isFieldOriented()) {
            chassisVelocity =
                    ChassisVelocity(
                            driveSignal.getTranslation().rotateBy(getPose().rotation.inverse()),
                            driveSignal.getRotation()
                    )
        } else {
            chassisVelocity =
                    ChassisVelocity(driveSignal.getTranslation(), driveSignal.getRotation())
        }

        val moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity)
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1.0)
        for (i in 0..moduleOutputs.size) {
            modules[i].set(moduleOutputs[i].length * 12.0, moduleOutputs[i].getAngle().toRadians())
        }
    }

    public fun getPoseAtTime(timestamp: Double): RigidTransform2 {
        synchronized(kinematicsLock) {
            if (latencyCompensationMap.isEmpty()) {
                return RigidTransform2.ZERO
            } else {
                return latencyCompensationMap.getInterpolated(InterpolatingDouble(timestamp))
            }
        }
    }

    public override fun update(time: Double, dt: Double) {
        updateOdometry(time, dt)

        lateinit var driveSignal: HolonomicDriveSignal
        val trajectorySignal =
                follower.update(getPose(), getVelocity(), getAngularVelocity(), time, dt)
        if (trajectorySignal.isPresent()) {
            driveSignal = trajectorySignal.get()
            driveSignal =
                    HolonomicDriveSignal(
                            driveSignal
                                    .getTranslation()
                                    .scale(1.0 / RobotController.getBatteryVoltage()),
                            driveSignal.getRotation() / RobotController.getBatteryVoltage(),
                            driveSignal.isFieldOriented()
                    )
        } else {
            synchronized(stateLock) { driveSignal = this.driveSignal }
        }

        updateModules(driveSignal, dt)
    }

    public override fun periodic() {
        val pose = getPose()
        odometryXEntry.setDouble(pose.translation.x)
        odometryYEntry.setDouble(pose.translation.y)
        odometryAngleEntry.setDouble(getPose().rotation.toDegrees())
    }

    public fun getFollower(): HolonomicMotionProfiledTrajectoryFollower {
        return follower
    }
}
