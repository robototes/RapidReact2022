package frc.team2412.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {

    public static class ClimbConstants {
        public static final double MAX_SPEED = 1;
        public static final double TEST_SPEED_EXTEND = 0.7;
        public static final double TEST_SPEED_RETRACT = -0.5;
        public static final double STOP_SPEED = 0;
        public static final double MAX_ENCODER_TICKS = 1000;
        public static final double MIN_ENCODER_TICKS = 0.8;
        public static final double RUNG_DISTANCE = 24; // inches
        public static final double GEARBOX_REDUCTION = 10.61;
        public static final double ENCODER_TICKS_PER_REVOLUTION = 2048;
        public static final double ARM_REACH_DISTANCE = Math.PI * RUNG_DISTANCE * GEARBOX_REDUCTION
                * ENCODER_TICKS_PER_REVOLUTION;

        public static final SupplyCurrentLimitConfiguration MOTOR_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                true, 40, 40, 500);

        enum HookArmState {
            ANGLED, UPRIGHT
        }

        enum ClimbSubsystemState {
            ENABLED, DISABLED
        }
    }

    private final WPI_TalonFX climbFixedMotor;
    private final WPI_TalonFX climbDynamicMotor;

    private final DoubleSolenoid solenoid;

    private static ClimbSubsystemState state = ClimbSubsystemState.DISABLED;

    public ClimbSubsystem(WPI_TalonFX climbFixedMotor, WPI_TalonFX climbDynamicMotor, DoubleSolenoid climbAngle) {
        setName("ClimbSubsystem");
        this.climbFixedMotor = climbFixedMotor;
        this.climbDynamicMotor = climbDynamicMotor;
        solenoid = climbAngle;

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.forwardSoftLimitEnable = true;
        motorConfig.reverseSoftLimitEnable = true;
        motorConfig.forwardSoftLimitThreshold = MAX_ENCODER_TICKS;
        motorConfig.reverseSoftLimitThreshold = MIN_ENCODER_TICKS;
        motorConfig.supplyCurrLimit = MOTOR_CURRENT_LIMIT;

        climbFixedMotor.configAllSettings(motorConfig);
        climbDynamicMotor.configAllSettings(motorConfig);
    }

    @Override
    public void periodic() {
        if (isDynamicFullyExtended() || isDynamicFullyRetracted()) {
            stopAngledArm();
        }
        if (isFixedFullyExtended() || isFixedFullyRetracted()) {
            stopFixedArm();
        }
    }

    public void setEnabled() {
        state = ClimbSubsystemState.ENABLED;
    }

    public void setDisabled() {
        state = ClimbSubsystemState.DISABLED;
    }

    public boolean getState() {
        return state == ClimbSubsystemState.ENABLED;
    }

    public void angleClimbHook(DoubleSolenoid.Value value) {
        solenoid.set(value);
    }

    public void extendFixedArm() {
        setMotor(ClimbConstants.TEST_SPEED_EXTEND, climbFixedMotor);
    }

    public void retractFixedArm() {
        setMotor(ClimbConstants.TEST_SPEED_RETRACT, climbFixedMotor);
    }

    public void stopFixedArm() {
        setMotor(ClimbConstants.STOP_SPEED, climbFixedMotor);
    }

    public void extendAngledArm() {
        setMotor(ClimbConstants.TEST_SPEED_EXTEND, climbDynamicMotor);
    }

    public void retractAngledArm() {
        setMotor(ClimbConstants.TEST_SPEED_RETRACT, climbDynamicMotor);
    }

    public void stopAngledArm() {
        setMotor(ClimbConstants.STOP_SPEED, climbDynamicMotor);
    }

    public void setMotor(double position, WPI_TalonFX motor) {
        if (state == ClimbSubsystemState.ENABLED) {
            motor.set(ControlMode.Position, position);
        }
    }

    public boolean isFixedFullyExtended() {
        return climbFixedMotor.getSelectedSensorPosition() >= ClimbConstants.MAX_ENCODER_TICKS;
    }

    public boolean isFixedFullyRetracted() {
        return climbFixedMotor.getSelectedSensorPosition() <= ClimbConstants.MIN_ENCODER_TICKS;
    }

    public boolean isDynamicFullyExtended() {
        return climbDynamicMotor.getSelectedSensorPosition() >= ClimbConstants.MAX_ENCODER_TICKS;
    }

    public boolean isDynamicFullyRetracted() {
        return climbDynamicMotor.getSelectedSensorPosition() <= ClimbConstants.MIN_ENCODER_TICKS;
    }

}