package frc.team2412.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Utility class for estimating the robot pose.
 *
 * <p>
 * Field coordinate system:
 * <ul>
 * <li>Origin is right corner of alliance wall</li>
 * <li>+X axis is away from alliance wall</li>
 * <li>+Y axis is left along alliance wall</li>
 * <li>0 rotation is away from alliance wall (along +X axis)</li>
 * <li>Positive rotation is counterclockwise</li>
 * </ul>
 * Equivalently, if your alliance wall is on the left:
 * <ul>
 * <li>Origin is bottom left</li>
 * <li>+X axis is right</li>
 * <li>+Y axis is up</li>
 * <li>0 rotation is right (along +X axis)</li>
 * <li>Positive rotation is counterclockwise</li>
 * </ul>
 * </p>
 */
public class VisionUtil {
    /**
     * Estimates the camera pose.
     *
     * @param targetPosition
     *            The position of the target in the field coordinate system.
     * @param targetDistance
     *            The distance from the camera to the target.
     * @param targetYaw
     *            The CCW-positive angle from the camera to the target. Note that Photon
     *            returns CW-positive.
     * @param cameraAngle
     *            The angle of the camera in the field coordinate system.
     * @return The estimated camera pose in the field coordinate system.
     */
    public static Pose2d estimateCameraPose(Translation2d targetPosition, double targetDistance, Rotation2d targetYaw,
            Rotation2d cameraAngle) {
        Translation2d cameraToTarget = new Translation2d(targetDistance, cameraAngle.plus(targetYaw));
        return new Pose2d(targetPosition.minus(cameraToTarget), cameraAngle);
    }

    /**
     * Estimates the field-centric robot pose.
     *
     * @param targetPosition
     *            The position of the target in the field coordinate system.
     * @param targetDistance
     *            The distance from the camera to the target.
     * @param targetYaw
     *            The CCW-positive angle from the camera to the target. Note that Photon returns
     *            CW-positive.
     * @param cameraAngle
     *            The angle of the camera in the field coordinate system.
     * @param cameraToRobot
     *            The transformation from the camera to the robot.
     * @return The robot pose in the field coordinate system.
     */
    public static Pose2d estimateRobotPose(Translation2d targetPosition, double targetDistance, Rotation2d targetYaw,
            Rotation2d cameraAngle, Transform2d cameraToRobot) {
        return estimateCameraPose(targetPosition, targetDistance, targetYaw, cameraAngle).transformBy(cameraToRobot);
    }

    /**
     * Calculates the transformation from the camera to the robot.
     *
     * @param robotCentricCameraPosition
     *            The position of the camera relative to the robot center. Note that the +X axis must be
     *            along {@code robotAngle}, and +Y axis along robotAngle is common.
     * @param robotToCameraAngle
     *            The CCW-positive angle from the robot to the camera.
     * @return The transformation from the camera to the robot.
     */
    public static Transform2d calculateCameraToRobot(Translation2d robotCentricCameraPosition,
            Rotation2d robotToCameraAngle) {
        return new Transform2d(robotCentricCameraPosition, robotToCameraAngle).inverse();
    }

    /**
     * Estimates the field-centric robot pose.
     *
     * @param targetPosition
     *            The position of the target in the field coordinate system.
     * @param targetDistance
     *            The distance from the camera to the target.
     * @param targetYaw
     *            The CCW-positive angle from the camera to the target. Note that Photon returns
     *            CW-positive.
     * @param robotAngle
     *            The angle of the robot in the field coordinate system.
     * @param robotToCameraAngle
     *            The CCW-positive angle from the robot to the camera.
     * @param robotCentricCameraPosition
     *            The position of the camera relative to the robot center. Note that the +X axis must be
     *            along {@code robotAngle}, and +Y axis along robotAngle is common.
     * @return The robot pose in the field coordinate system.
     */
    public static Pose2d estimateRobotPose(Translation2d targetPosition, double targetDistance, Rotation2d targetYaw,
            Rotation2d robotAngle, Rotation2d robotToCameraAngle, Translation2d robotCentricCameraPosition) {
        Translation2d cameraToTarget = new Translation2d(targetDistance,
                robotAngle.plus(robotToCameraAngle).plus(targetYaw));
        Translation2d cameraPosition = targetPosition.minus(cameraToTarget);
        Translation2d cameraToRobot = robotCentricCameraPosition.rotateBy(robotAngle);
        return new Pose2d(cameraPosition.minus(cameraToRobot), robotAngle);
    }
}
