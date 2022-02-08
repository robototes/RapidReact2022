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
        public static final int frontLeftDriveMotorPort = 0;
        public static final int rearLeftDriveMotorPort = 2;
        public static final int frontRightDriveMotorPort = 4;
        public static final int rearRightDriveMotorPort = 6;

        public static final int frontLeftTurningMotorPort = 1;
        public static final int rearLeftTurningMotorPort = 3;
        public static final int frontRightTurningMotorPort = 5;
        public static final int rearRightTurningMotorPort = 7;

        public static final int[] frontLeftTurningEncoderPorts = new int[] { 0, 1 };
        public static final int[] rearLeftTurningEncoderPorts = new int[] { 2, 3 };
        public static final int[] frontRightTurningEncoderPorts = new int[] { 4, 5 };
        public static final int[] rearRightTurningEncoderPorts = new int[] { 5, 6 };

        public static final boolean frontLeftTurningEncoderReversed = false;
        public static final boolean rearLeftTurningEncoderReversed = true;
        public static final boolean frontRightTurningEncoderReversed = false;
        public static final boolean rearRightTurningEncoderReversed = true;

        public static final int[] frontLeftDriveEncoderPorts = new int[] { 7, 8 };
        public static final int[] rearLeftDriveEncoderPorts = new int[] { 9, 10 };
        public static final int[] frontRightDriveEncoderPorts = new int[] { 11, 12 };
        public static final int[] rearRightDriveEncoderPorts = new int[] { 13, 14 };

        public static final boolean frontLeftDriveEncoderReversed = false;
        public static final boolean rearLeftDriveEncoderReversed = true;
        public static final boolean frontRightDriveEncoderReversed = false;
        public static final boolean rearRightDriveEncoderReversed = true;

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

    public static final class ModuleConstants {

    }

    // todod someone must find these values
    public static final class AutoConstants {
        public static final double MaxSpeedMetersPerSecond = 3;
        public static final double MaxAccelerationMetersPerSecondSquared = 3;
        public static final double MaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double MaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double PXController = 0.1;
        public static final double PYController = 0.1;
        public static final double PThetaController = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                MaxAngularSpeedRadiansPerSecond, MaxAngularSpeedRadiansPerSecondSquared);

        public static final double maxSpeedMetersPerSecond = 0.1;
        public static final double maxAccelerationMetersPerSecondSquared = 0.3;
        public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;

    }

    public static final class OIConstants {

    }

}
