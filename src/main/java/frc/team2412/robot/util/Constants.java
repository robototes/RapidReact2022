package frc.team2412.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static class DriveConstants {

        public static final double trackWidth = 1.0;
        // Distance between centers of right and left wheels on robot
        public static final double wheelBase = 1.0;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2, trackWidth / 2),
                new Translation2d(wheelBase / 2, -trackWidth / 2),
                new Translation2d(-wheelBase / 2, trackWidth / 2),
                new Translation2d(-wheelBase / 2, -trackWidth / 2));

    }

    // todod someone must find these values


}
