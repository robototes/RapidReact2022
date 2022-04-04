package frc.team2412.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class GeoConvertor {
    public static Pose2d rigidInchesToPoseMeters(RigidTransform2 trans) {
        return new Pose2d(Units.inchesToMeters(trans.translation.x), Units.inchesToMeters(trans.translation.y),
                rotation2toRotation2d(trans.rotation));
    }

    public static Pose2d rigidMetersToPoseInches(RigidTransform2 trans) {
        return new Pose2d(Units.metersToInches(trans.translation.x), Units.metersToInches(trans.translation.y),
                rotation2toRotation2d(trans.rotation));
    }

    public static RigidTransform2 poseToRigid(Pose2d pose) {
        return new RigidTransform2(translation2dToVector2(pose.getTranslation()),
                rotation2dToRotation2(pose.getRotation()));
    }

    public static Rotation2d rotation2toRotation2d(Rotation2 r) {
        return new Rotation2d(r.cos, r.sin);
    }

    public static Rotation2 rotation2dToRotation2(Rotation2d r) {
        return new Rotation2(r.getCos(), r.getSin(), true);
    }

    public static Vector2 translation2dToVector2(Translation2d trans) {
        return new Vector2(trans.getX(), trans.getY());
    }
}
