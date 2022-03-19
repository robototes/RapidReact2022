package frc.team2412.robot.subsystem;

import static frc.team2412.robot.Hardware.*;

import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Robot;
import frc.team2412.robot.util.GeoConvertor;
import frc.team2412.robot.util.PFFController;
import frc.team2412.robot.util.VectorSlewLimiter;
import org.frcteam2910.common.control.*;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.drivers.Pigeon;
import org.frcteam2910.common.util.*;

import java.util.Map;
import java.util.Optional;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystem.DrivebaseSubsystem.DriveConstants.*;

public class DrivebaseSubsystem extends SubsystemBase implements UpdateManager.Updatable {

    // TODO find values as these are just copied from 2910
    public static class DriveConstants {

        public static final double TRACKWIDTH = 1.0;
        public static final double WHEELBASE = 1.0;

        public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
                0.072746,
                0.0032181,
                0.30764);

        // these values need to be found
        public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
                new FeedforwardConstraint(3.0, FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                        FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false), // old value was 11.0
                new MaxAccelerationConstraint(3.0), // old value was 12.5 * 12.0
                new CentripetalAccelerationConstraint(3.0), // old value was 15 * 12.0
                // in inches
                new FeedforwardConstraint(11.0, FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                        FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false),
                new MaxAccelerationConstraint(3),
                new CentripetalAccelerationConstraint(15 * 12.0)
        };

        public static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;

        public static final boolean ANTI_TIP_DEFAULT = true;

        public static final boolean FIELD_CENTRIC_DEFAULT = true;

        public static final double TIP_P = 0.05, TIP_F = 0, TIP_TOLERANCE = 10, ACCEL_LIMIT = 4;
    }

    private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(0.1, 0.0, 0.0),
            new PidConstants(0, 0.0, 0.0),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

    private final SwerveKinematics swerveKinematics = new SwerveKinematics(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // front left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // front right
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // back left
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // back right
    );

    private final SwerveDriveKinematics wpi_driveKinematics = new SwerveDriveKinematics(
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // front left
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // front right
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // back left
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // back right
    );

    private final SwerveModule[] modules;
    private final double moduleMaxVelocityMetersPerSec;

    private final Object sensorLock = new Object();
    @GuardedBy("sensorLock")
    private final Gyroscope gyroscope;

    private final Object kinematicsLock = new Object();
    @GuardedBy("kinematicsLock")
    private final SwerveOdometry swerveOdometry = new SwerveOdometry(swerveKinematics, RigidTransform2.ZERO);
    @GuardedBy("kinematicsLock")
    private RigidTransform2 pose = RigidTransform2.ZERO;
    @GuardedBy("kinematicsLock")
    private final InterpolatingTreeMap<InterpolatingDouble, RigidTransform2> latencyCompensationMap = new InterpolatingTreeMap<>();
    @GuardedBy("kinematicsLock")
    private Vector2 velocity = Vector2.ZERO;
    @GuardedBy("kinematicsLock")
    private double angularVelocity = 0.0;

    private final Object stateLock = new Object();
    @GuardedBy("stateLock")
    private HolonomicDriveSignal driveSignal = null;

    // Logging
    private final NetworkTableEntry odometryXEntry;
    private final NetworkTableEntry odometryYEntry;
    private final NetworkTableEntry odometryAngleEntry;
    private final NetworkTableEntry speedModifier;
    private final NetworkTableEntry antiTip;
    private final NetworkTableEntry fieldCentric;

    private final Field2d field = new Field2d();

    private final PFFController<Vector2> tipController;
    private final VectorSlewLimiter accelLimiter;

    public DrivebaseSubsystem() {
        boolean comp = Robot.getInstance().isCompetition();

        synchronized (sensorLock) {
            gyroscope = comp ? new Pigeon(GYRO_PORT) : new NavX(SerialPort.Port.kMXP);
            if (gyroscope instanceof Pigeon)
                gyroscope.setInverted(true);
            SmartDashboard.putData("Field", field);
        }

        ShuffleboardTab tab = Shuffleboard.getTab("Drivebase");

        boolean supportAbsoluteEncoder = comp && !Robot.isSimulation();
        modules = new SwerveModule[] { FRONT_LEFT_CONFIG.create(supportAbsoluteEncoder),
                FRONT_RIGHT_CONFIG.create(supportAbsoluteEncoder),
                BACK_LEFT_CONFIG.create(supportAbsoluteEncoder),
                BACK_RIGHT_CONFIG.create(supportAbsoluteEncoder) };
        moduleMaxVelocityMetersPerSec = MODULE_MAX_VELOCITY_METERS_PER_SEC;

        odometryXEntry = tab.add("X", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        odometryYEntry = tab.add("Y", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        odometryAngleEntry = tab.add("Angle", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();
        tab.addNumber("Trajectory X", () -> {
            if (follower.getLastState() == null) {
                return 0.0;
            }
            return follower.getLastState().getPathState().getPosition().x;
        })
                .withPosition(1, 0)
                .withSize(1, 1);
        tab.addNumber("Trajectory Y", () -> {
            if (follower.getLastState() == null) {
                return 0.0;
            }
            return follower.getLastState().getPathState().getPosition().y;
        })
                .withPosition(1, 1)
                .withSize(1, 1);

        tab.addNumber("Rotation Voltage", () -> {
            HolonomicDriveSignal signal;
            synchronized (stateLock) {
                signal = driveSignal;
            }

            if (signal == null) {
                return 0.0;
            }

            return signal.getRotation() * RobotController.getBatteryVoltage();
        });

        speedModifier = tab.add("Speed Modifier", 1f)
                .withPosition(2, 1)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 1.0))
                .getEntry();

        tab.addNumber("Average Velocity", this::getAverageAbsoluteValueVelocity);

        antiTip = tab.add("Anti Tip", ANTI_TIP_DEFAULT)
                .withPosition(3, 1)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        fieldCentric = tab.add("Field Centric", FIELD_CENTRIC_DEFAULT)
                .withPosition(3, 2)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();

        tipController = PFFController.ofVector2(TIP_P, TIP_F).setTargetPosition(getGyroscopeXY())
                .setTargetPositionTolerance(TIP_TOLERANCE);

        accelLimiter = new VectorSlewLimiter(ACCEL_LIMIT);
    }

    public Vector2 getGyroscopeXY() {
        synchronized (sensorLock) {
            if (gyroscope instanceof Pigeon)
                return new Vector2(-((Pigeon) gyroscope).getAxis(Pigeon.Axis.PITCH),
                        ((Pigeon) gyroscope).getAxis(Pigeon.Axis.ROLL)).scale(180 / Math.PI);
            if (gyroscope instanceof NavX)
                return new Vector2(((NavX) gyroscope).getAxis(NavX.Axis.ROLL),
                        ((NavX) gyroscope).getAxis(NavX.Axis.PITCH)).scale(180 / Math.PI);
        }
        return Vector2.ZERO;
    }

    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose;
        }
    }

    public Pose2d getPoseAsPoseMeters() {
        synchronized (kinematicsLock) {
            return GeoConvertor.rigidInchesToPoseMeters(pose);
        }
    }

    public Vector2 getVelocity() {
        synchronized (kinematicsLock) {
            return velocity;
        }
    }

    public double getAngularVelocity() {
        synchronized (kinematicsLock) {
            return angularVelocity;
        }
    }

    public Rotation2 getAngle() {
        synchronized (kinematicsLock) {
            return getPose().rotation;
            // return Robot.getInstance().isCompetition() ? getPose().rotation.inverse() : getPose().rotation;
        }
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
        synchronized (stateLock) {
            driveSignal = new HolonomicDriveSignal(translationalVelocity.scale(speedModifier.getDouble(1.0)),
                    rotationalVelocity * speedModifier.getDouble(1.0), isFieldOriented);
        }
    }

    public void resetPose(Pose2d pose) {
        synchronized (kinematicsLock) {
            this.pose = GeoConvertor.poseToRigid(pose);
            swerveOdometry.resetPose(this.pose);
        }
    }

    public void resetPose(RigidTransform2 pose) {
        synchronized (kinematicsLock) {
            this.pose = pose;
            swerveOdometry.resetPose(pose);
            resetGyroAngle(Rotation2.ZERO);
        }
    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            gyroscope.setAdjustmentAngle(
                    gyroscope.getUnadjustedAngle().rotateBy(angle.inverse()));
            tipController.setTargetPosition(getGyroscopeXY());
        }
    }

    public double getAverageAbsoluteValueVelocity() {
        double averageVelocity = 0;
        for (var module : modules) {
            averageVelocity += Math.abs(module.getDriveVelocity());
        }
        return averageVelocity / 4;
    }

    private void updateOdometry(double time, double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getSteerAngle()))
                    .scale(module.getDriveVelocity() * 39.37008);
        }

        Rotation2 angle;
        double angularVelocity;
        synchronized (sensorLock) {
            angle = gyroscope.getAngle();
            // angle = (angle.toDegrees() < 0) ? Rotation2.fromDegrees(360 + angle.toDegrees()) : angle;
        }

        ChassisVelocity velocity = swerveKinematics.toChassisVelocity(moduleVelocities);

        synchronized (kinematicsLock) {

            this.pose = swerveOdometry.update(angle, dt, moduleVelocities);
            if (latencyCompensationMap.size() > MAX_LATENCY_COMPENSATION_MAP_ENTRIES) {
                latencyCompensationMap.remove(latencyCompensationMap.firstKey());
            }
            latencyCompensationMap.put(new InterpolatingDouble(time), pose);
            this.velocity = velocity.getTranslationalVelocity();
            this.angularVelocity = velocity.getAngularVelocity();
        }
    }

    public void updateModules(HolonomicDriveSignal driveSignal) {
        ChassisVelocity chassisVelocity;
        if (driveSignal == null) {
            chassisVelocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (driveSignal.isFieldOriented()) {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation().rotateBy(getAngle()),
                    driveSignal.getRotation());
        } else {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation(),
                    driveSignal.getRotation());
        }

        Vector2[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);
        for (int i = 0; i < moduleOutputs.length; i++) {
            var module = modules[i];
            module.set(moduleOutputs[i].length * 12.0, moduleOutputs[i].getAngle().toRadians());
        }
    }

    public void updateModules(ChassisSpeeds chassisSpeeds) {
        HolonomicDriveSignal holonomicDriveSignal = new HolonomicDriveSignal(
                new Vector2(
                        (chassisSpeeds.vxMetersPerSecond / moduleMaxVelocityMetersPerSec),
                        (chassisSpeeds.vyMetersPerSecond / moduleMaxVelocityMetersPerSec)),
                chassisSpeeds.omegaRadiansPerSecond, true);

        synchronized (stateLock) {
            this.driveSignal = holonomicDriveSignal;
        }
    }

    public RigidTransform2 getPoseAtTime(double timestamp) {
        synchronized (kinematicsLock) {
            if (latencyCompensationMap.isEmpty()) {
                return RigidTransform2.ZERO;
            }
            return latencyCompensationMap.getInterpolated(new InterpolatingDouble(timestamp));
        }
    }

    @Override
    public void update(double time, double dt) {
        updateOdometry(time, dt);

        HolonomicDriveSignal signal;
        Optional<HolonomicDriveSignal> trajectorySignal = follower.update(
                getPose(),
                getVelocity(),
                getAngularVelocity(),
                time,
                dt);
        if (trajectorySignal.isPresent()) {
            signal = trajectorySignal.get();
            signal = new HolonomicDriveSignal(
                    signal.getTranslation().scale(1.0 / RobotController.getBatteryVoltage()),
                    signal.getRotation() / RobotController.getBatteryVoltage(),
                    signal.isFieldOriented());
        } else {
            synchronized (stateLock) {
                if (getAntiTip() && driveSignal != null) {
                    signal = new HolonomicDriveSignal( // create updated drive signal
                            accelLimiter.calculate(driveSignal.getTranslation()) // vector accel limiter
                                    .rotateBy(driveSignal.isFieldOriented() ? // flatten
                                            getAngle() : Rotation2.ZERO) // same code as other block
                                    .add(tipController.update(getGyroscopeXY())), // anti tip stuff
                            driveSignal.getRotation(), false); // retain rotation
                } else
                    signal = driveSignal;
            }
        }
        updateModules(signal);
    }

    @Override
    public void periodic() {
        // Pose2d pose = getPoseAsPoseMeters();
        synchronized (kinematicsLock) {
            odometryXEntry.setDouble(pose.translation.x);
            odometryYEntry.setDouble(pose.translation.y);
            odometryAngleEntry.setDouble(pose.rotation.toDegrees());
        }
        // System.out.println(pose);
        Pose2d pose = getPoseAsPoseMeters();
        field.setRobotPose(pose);

    }

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }

    public void follow(Path p) {
        follower.follow(new Trajectory(p, TRAJECTORY_CONSTRAINTS, 12.0));
    }

    public boolean getFieldCentric() {
        return fieldCentric.getBoolean(false);
    }

    public boolean getAntiTip() {
        return antiTip.getBoolean(false);
    }
}
