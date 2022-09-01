package frc.team2412.robot.subsystem;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

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
    
}
