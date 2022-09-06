package frc.team2412.robot.subsystem;

import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.FeedforwardConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.TrajectoryConstraint;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Constants {

    public static class ClimbConstants {
        // Climb dynamic motor speeds
        public static final double EXTEND_SPEED = 0.15;
        public static final double RETRACT_SPEED = -0.15;

        // PID stuff
        public static final int PID_EXTENSION_SLOT = 0;
        public static final double EXTENSION_P = 0.5;
        public static final double EXTENSION_I = 0;
        public static final double EXTENSION_D = 0;
        public static final double EXTENSION_F = 0;

        public static final int PID_RETRACTION_SLOT = 1;
        public static final double RETRACTION_P = 0.5; // TODO: figure out values
        public static final double RETRACTION_I = 0;
        public static final double RETRACTION_D = 0;
        public static final double RETRACTION_F = 0.18;
        // This is based on the minimum amount of motor power need to keep climb arm in place, need to test

        // Relating to physical climb structure things
        // was previously mid
        public static final double MID_RUNG_HEIGHT = 5.5;
        public static final double RETRACT_HEIGHT = 0.166;

        public static final double CLIMB_OFFSET = 4.75;

        // Doing integer division, which returns 11757 (previously 8789)
        // Probably should do floating point division, which returns 11759.3
        public static final double ENCODER_TICKS_PER_REMY = ((272816.0 / 58) * 2 * 5) / 4 * 6;

        // Motor current limit config
        public static final SupplyCurrentLimitConfiguration MOTOR_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                true, 40, 60, 15);
    }

    public static class IndexConstants {

        public static final double CURRENT_LIMIT_TRIGGER_SECONDS = 0.5;
        public static final double CURRENT_LIMIT_RESET_AMPS = 10;
        public static final double CURRENT_LIMIT_TRIGGER_AMPS = 20;

        // Index Motor Speeds

        public static final double INDEX_FEEDER_SPEED = 0.125;
        public static final double INDEX_FEEDER_SHOOT_SPEED = 0.2;
        public static final double INDEX_INGEST_SHOOT_SPEED = 0.2;

        public static final double INDEX_IN_SPEED = IntakeConstants.INTAKE_IN_SPEED / 2;
        public static final double INDEX_OUT_SPEED = -0.3;

        // The current limit
        public static final SupplyCurrentLimitConfiguration MAX_MOTOR_CURRENT = new SupplyCurrentLimitConfiguration(
                true, CURRENT_LIMIT_RESET_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TRIGGER_SECONDS);

    }

    public static class ShooterVisionConstants {
        // Dimensions are in inches
        public static final double LIMELIGHT_HEIGHT_OFFSET = 37.5;
        public static final double RIM_HEIGHT = 104; // 8ft8in
        public static final double HEIGHT_TO_RIM = RIM_HEIGHT - LIMELIGHT_HEIGHT_OFFSET;
        public static final double HUB_RADIUS = 24;
        // Angles are in degrees
        public static final double LIMELIGHT_ANGLE_OFFSET = Math.toDegrees(Math.atan2(HEIGHT_TO_RIM, 360 - HUB_RADIUS)); // 10.95

        public static final int COMP_PIPELINE_NUM = 5;
        // -0.766666 limelight crosshair offset (3/19 update)
    }

    // Constants
    public static class IntakeConstants {

        public static final double INNER_INTAKE_IN_SPEED = 0.35; // TODO
        public static final double INTAKE_IN_SPEED = 0.85;
        public static final double INTAKE_OUT_SPEED = -0.3;

        public static final SupplyCurrentLimitConfiguration MAX_MOTOR_CURRENT = new SupplyCurrentLimitConfiguration(
                true, 20, 20, 1);

        // Enums

        public static enum IntakeSolenoidState {
            EXTEND(DoubleSolenoid.Value.kForward, "Extended"), RETRACT(DoubleSolenoid.Value.kReverse, "Reversed");

            public final DoubleSolenoid.Value value;
            public final String state;

            private IntakeSolenoidState(DoubleSolenoid.Value value, String state) {
                this.value = value;
                this.state = state;
            }
        }

    }

    public static class DriveConstants {

        public static final double TRACKWIDTH = 1.0;
        public static final double WHEELBASE = 1.0;

        public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
                0.0593, // velocity
                0.00195, // acceleration
                0.236); // static

        public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
                new FeedforwardConstraint(11.0, FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                        FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false), // old value was 11.0
                new MaxAccelerationConstraint(3 * 12.0), // old value was 12.5 * 12.0
                new MaxVelocityConstraint(4 * 12.0),
                new CentripetalAccelerationConstraint(6 * 12.0), // old value was 15 * 12.0
        };

        public static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;

        public static final boolean ANTI_TIP_DEFAULT = true;

        public static final boolean FIELD_CENTRIC_DEFAULT = true;

        public static final double TIP_P = 0.05, TIP_F = 0, TIP_TOLERANCE = 10, ACCEL_LIMIT = 4;

        public static final Rotation2 PRACTICE_BOT_DRIVE_OFFSET = Rotation2.fromDegrees(-90), // should be 90
                COMP_BOT_DRIVE_OFFSET = Rotation2.fromDegrees(0);
    }
}
