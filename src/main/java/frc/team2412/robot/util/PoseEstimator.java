package frc.team2412.robot.util;

import static frc.team2412.robot.util.PoseEstimator.PoseEstimationConstants.*;

import java.util.function.DoubleSupplier;

import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class PoseEstimator {
    public static class PoseEstimationConstants {
        // Dimensions are in inches
        // Origin is hub, positive X is right looking from the driver station, positive Y is away from
        // driver station
        public static final Vector2 STARTING_ROBOT_POSITION = new Vector2(5 * 12, 5 * 12);
        // Rotation2 can be specified by degrees or radians
        // STARTING_ROBOT_ROTATION of 0 means straight forward from the driver station
        public static final Rotation2 STARTING_ROBOT_ROTATION = Rotation2.ZERO; // Placeholder
    }

    // Singleton stuff
    private static PoseEstimator instance = null;

    public static PoseEstimator getInstance(ShooterVision vision, Gyroscope gyro, DoubleSupplier turretAngleSupplier) {
        if (instance == null) {
            instance = new PoseEstimator(vision, gyro, turretAngleSupplier);
        }
        return instance;
    }

    private final ShooterVision vision;
    private final Gyroscope gyro;
    private final Rotation2 gyroAdjustmentAngle;
    private final DoubleSupplier turretAngleSupplier;

    private PoseEstimator(ShooterVision vision, Gyroscope gyro, DoubleSupplier turretAngleSupplier) {
        this.vision = vision;
        this.gyro = gyro;
        this.gyroAdjustmentAngle = STARTING_ROBOT_ROTATION.rotateBy(gyro.getUnadjustedAngle().inverse());
        this.turretAngleSupplier = turretAngleSupplier;
    }

    /**
     * Returns the estimated robot pose as a {@link RigidTransform2}.
     *
     * The translation (inches) is relative to the hub, and the rotation is relative to straight forward
     * from the driver station. Positive rotation is clockwise, negative is counterclockwise.
     *
     * @return The estimated robot pose as a {@link RigidTransform2}.
     */
    public RigidTransform2 getVisionPoseRelativeToHub() {
        double distanceToHub = vision.getDistance();
        Rotation2 fieldCentricRobotAngle = gyro.getUnadjustedAngle().rotateBy(gyroAdjustmentAngle);
        Rotation2 robotCentricAngleToHub = Rotation2.fromDegrees(vision.getYaw() + turretAngleSupplier.getAsDouble());
        Rotation2 fieldCentricRobotToHubAngle = fieldCentricRobotAngle.rotateBy(robotCentricAngleToHub);
        Rotation2 fieldCentricHubToRobotAngle = fieldCentricRobotToHubAngle.rotateBy(Rotation2.fromDegrees(180));
        Vector2 translation = Vector2.fromAngle(fieldCentricHubToRobotAngle).scale(distanceToHub);
        return new RigidTransform2(translation, fieldCentricRobotAngle);
    }

    /**
     * Returns the estimated robot pose relative to the starting pose as a {@link RigidTransform2}.
     *
     * The translation (inches) is relative to the starting position, and the rotation is relative to
     * the starting rotation. Positive is clockwise, negative is counterclockwise.
     *
     * @return The estimated robot pose relative to the start as a {@link RigidTransform2}.
     */
    public RigidTransform2 getVisionPoseRelativeToStart() {
        RigidTransform2 poseRelativeToHub = getVisionPoseRelativeToHub();
        Rotation2 rotation = poseRelativeToHub.rotation.rotateBy(STARTING_ROBOT_ROTATION.inverse());
        Vector2 translation = poseRelativeToHub.translation.subtract(STARTING_ROBOT_POSITION);
        return new RigidTransform2(translation, rotation);
    }
}
