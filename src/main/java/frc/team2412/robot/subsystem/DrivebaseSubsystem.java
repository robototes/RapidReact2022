package frc.team2412.robot.subsystem;

import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystem.Constants.DriveConstants.*;

import java.util.*;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.kinematics.*;
import org.frcteam2910.common.math.*;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.drivers.*;
import org.frcteam2910.common.util.*;
import org.frcteam2910.common.util.InterpolatingTreeMap;

import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.*;
import frc.team2412.robot.util.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class DrivebaseSubsystem extends SubsystemBase implements UpdateManager.Updatable, Loggable {

    private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(0.0708, 0.0, 0.0),
            new PidConstants(5.0, 0.0, 0.0),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

    private final SwerveKinematics swerveKinematics = new SwerveKinematics(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // front left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // front right
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // back left
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // back right
    );

    private SwerveModule[] modules;

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
    @Log(name = "X", columnIndex = 0, rowIndex = 0)
    private double odometryXEntry;
    @Log(name = "Y", columnIndex = 0, rowIndex = 1)
    private double odometryYEntry;
    @Log(name = "Angle", columnIndex = 0, rowIndex = 2)
    private double odometryAngleEntry;
    
    @Config.NumberSlider(name = "Speed Modifier", columnIndex = 2, rowIndex = 1, defaultValue = 0.95)
    private double speedModifier;
   
    @Config.NumberSlider(name = "Shoot Speed", columnIndex = 4, rowIndex = 1, defaultValue = 0.6)
    private double shootSpeed;
    @Config.ToggleSwitch(name = "Shoot speed toggled", columnIndex = 4, rowIndex = 3)
    private boolean shootSpeedToggle;
   
    @Config.ToggleSwitch(name = "Anti tip", columnIndex = 3, rowIndex = 1, width = 2, defaultValue = ANTI_TIP_DEFAULT)
    private boolean antiTip;
    @Config.ToggleSwitch(name = "Field Centric", columnIndex = 2, rowIndex = 2, width = 2, defaultValue = FIELD_CENTRIC_DEFAULT)
    private boolean fieldCentric;
    
    @Config(name = "Set Pose X", columnIndex = 5, rowIndex = 3)
    private double poseSetX;
    @Config(name = "Set Pose Y", columnIndex = 6, rowIndex = 3)
    private double poseSetY;
    @Config(name = "Set Pose Angle", columnIndex = 6, rowIndex = 3)
    private double poseSetAngle;
   
    @Log(name = "Trajectory X", columnIndex = 1, rowIndex = 0)
    private double trajectoryX;
    @Log(name = "Trajectory Y", columnIndex = 1, rowIndex = 1)
    private double trajectoryY;
    @Log(name = "Rotation Voltage", columnIndex = 1, rowIndex = 1)
    private double rotationVoltage;
    
    

    private final Field2d field = new Field2d();

    private final PFFController<Vector2> tipController;
    private final VectorSlewLimiter accelLimiter;

    public DrivebaseSubsystem() {
        setName("Drivebase");
        boolean comp = Robot.getInstance().isCompetition();

        synchronized (sensorLock) {
            gyroscope = comp ? new PigeonTwo(GYRO_PORT, Hardware.DRIVETRAIN_INTAKE_CAN_BUS_NAME)
                    : new NavX(SerialPort.Port.kMXP);
            if (gyroscope instanceof PigeonTwo)
                gyroscope.setInverted(true);
            SmartDashboard.putData("Field", field);
        }

        boolean supportAbsoluteEncoder = comp && !Robot.isSimulation();
        modules = new SwerveModule[] { FRONT_LEFT_CONFIG.create(supportAbsoluteEncoder),
                FRONT_RIGHT_CONFIG.create(supportAbsoluteEncoder),
                BACK_LEFT_CONFIG.create(supportAbsoluteEncoder),
                BACK_RIGHT_CONFIG.create(supportAbsoluteEncoder) };

        tipController = PFFController.ofVector2(TIP_P, TIP_F).setTargetPosition(getGyroscopeXY())
                .setTargetPositionTolerance(TIP_TOLERANCE);

        accelLimiter = new VectorSlewLimiter(ACCEL_LIMIT);
    }

    @Override
    public String configureLogName() {
        return "Drivebase";
    }

    public Vector2 getGyroscopeXY() {
        synchronized (sensorLock) {
            if (gyroscope instanceof PigeonTwo)
                return new Vector2(-((PigeonTwo) gyroscope).getAxis(PigeonTwo.Axis.PITCH),
                        ((PigeonTwo) gyroscope).getAxis(PigeonTwo.Axis.ROLL)).scale(180 / Math.PI);
            if (gyroscope instanceof NavX)
                return new Vector2(((NavX) gyroscope).getAxis(NavX.Axis.ROLL),
                        ((NavX) gyroscope).getAxis(NavX.Axis.PITCH)).scale(180 / Math.PI);
        }
        return Vector2.ZERO;
    }

    public Rotation2 getGyroscopeUnadjustedAngle() {
        synchronized (sensorLock) {
            return gyroscope.getUnadjustedAngle();
        }
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

    public void setPose() {
        resetPose(new Pose2d(poseSetX, poseSetY,
                Rotation2d.fromDegrees(poseSetAngle)));
        resetGyroAngle(Rotation2.fromDegrees(poseSetAngle).inverse());
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
            return Robot.getInstance().isCompetition() ? getPose().rotation.inverse()
                    : getPose().rotation;
        }
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity) {
        synchronized (stateLock) {
            driveSignal = new HolonomicDriveSignal(
                    translationalVelocity.scale(speedModifier)
                            .scale(shootSpeedToggle ? shootSpeed : 1)
                            .rotateBy(getRotationAdjustment()),
                    (rotationalVelocity * speedModifier), false);

        }
    }

    public void resetPose(Pose2d pose) {
        synchronized (kinematicsLock) {
            this.pose = GeoConvertor.poseToRigid(pose);
            resetGyroAngle(GeoConvertor.rotation2dToRotation2(pose.getRotation()));
            swerveOdometry.resetPose(this.pose);
        }
    }

    public void resetPose(RigidTransform2 pose) {
        synchronized (kinematicsLock) {
            this.pose = pose;
            resetGyroAngle(pose.rotation);
            swerveOdometry.resetPose(pose);
        }
    }

    @Config(tabName = "Drivebase", name = "reset modules", columnIndex = 5, rowIndex = 0)
    public void resetModules(boolean reset) {
        if (reset) {
            boolean supportAbsoluteEncoder = Robot.getInstance().isCompetition() && !Robot.isSimulation();

            modules = new SwerveModule[] { FRONT_LEFT_CONFIG.create(supportAbsoluteEncoder),
                    FRONT_RIGHT_CONFIG.create(supportAbsoluteEncoder),
                    BACK_LEFT_CONFIG.create(supportAbsoluteEncoder),
                    BACK_RIGHT_CONFIG.create(supportAbsoluteEncoder) };
        }

    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            gyroscope.setAdjustmentAngle(
                    gyroscope.getUnadjustedAngle().rotateBy(angle.inverse()));
            tipController.setTargetPosition(getGyroscopeXY());
        }
    }

    // this returns a value that can be rotated to the pose to make the intake the front of the robot
    private Rotation2 getRotationAdjustment() {
        return !Robot.getInstance().isCompetition() ? PRACTICE_BOT_DRIVE_OFFSET
                : COMP_BOT_DRIVE_OFFSET;
    }

    @Log(name = "Average Velocity", columnIndex = 8, rowIndex = 3)
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
        synchronized (sensorLock) {
            angle = gyroscope.getAngle().inverse();
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
        } else if (fieldCentric) {
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

        if (driveSignal != null && driveSignal.getTranslation().length <= 0.01 && driveSignal.getRotation() == 0) {
            setToX();
            return;
        }

        for (int i = 0; i < moduleOutputs.length; i++) {
            var module = modules[i];
            module.set(moduleOutputs[i].length * 12.0, moduleOutputs[i].getAngle().toRadians());
        }
    }

    public void setToX() {
        modules[0].set(0, Math.toRadians(45));
        modules[1].set(0, Math.toRadians(-45));
        modules[2].set(0, Math.toRadians(-45));
        modules[3].set(0, Math.toRadians(45));
    }

    public void updateModules(ChassisSpeeds chassisSpeeds) {
        HolonomicDriveSignal holonomicDriveSignal = new HolonomicDriveSignal(
                new Vector2(
                        (chassisSpeeds.vxMetersPerSecond / MODULE_MAX_VELOCITY_METERS_PER_SEC),
                        (chassisSpeeds.vyMetersPerSecond / MODULE_MAX_VELOCITY_METERS_PER_SEC)),
                chassisSpeeds.omegaRadiansPerSecond, false);

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
                                            getAngle().inverse() : Rotation2.ZERO) // same code as other block
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
        synchronized (kinematicsLock) {
            odometryXEntry = Units.inchesToMeters(pose.translation.x);
            odometryYEntry = Units.inchesToMeters(pose.translation.y);
            odometryAngleEntry = pose.rotation.toDegrees();
        }

        synchronized (stateLock) {
            if (driveSignal != null) {
                rotationVoltage = driveSignal.getRotation() * RobotController.getBatteryVoltage();
            }
        }

        if (follower.getLastState() != null) {
            trajectoryX = follower.getLastState().getPathState().getPosition().x;
            trajectoryY = follower.getLastState().getPathState().getPosition().y;
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
        return fieldCentric;
    }

    public boolean getAntiTip() {
        return antiTip;
    }
}
