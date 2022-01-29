package frc.team2412.robot.subsystem;

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
        public static final double MIN_ENCODER_TICKS = 0;

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
        double positionDynamic = climbDynamicMotor.getSelectedSensorPosition();
        if (positionDynamic >= MAX_ENCODER_TICKS || positionDynamic <= MIN_ENCODER_TICKS) {
            stopAngledArm();
        }
        double positionFixed = climbFixedMotor.getSelectedSensorPosition();
        if (positionFixed >= MAX_ENCODER_TICKS || positionFixed <= MIN_ENCODER_TICKS) {
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

    public void angleClimbHook(boolean extended) {
        solenoid.set(extended ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public void extendFixedArm() {
        if (state == ClimbSubsystemState.ENABLED)
            climbFixedMotor.set(ClimbConstants.TEST_SPEED_EXTEND);
    }

    public void retractFixedArm() {
        if (state == ClimbSubsystemState.ENABLED)
            climbFixedMotor.set(ClimbConstants.TEST_SPEED_RETRACT);
    }

    public void stopFixedArm() {
        if (state == ClimbSubsystemState.ENABLED)
            climbFixedMotor.set(ClimbConstants.STOP_SPEED);
    }

    public void extendAngledArm() {
        if (state == ClimbSubsystemState.ENABLED)
            climbDynamicMotor.set(ClimbConstants.TEST_SPEED_EXTEND);
    }

    public void retractAngledArm() {
        if (state == ClimbSubsystemState.ENABLED)
            climbDynamicMotor.set(ClimbConstants.TEST_SPEED_RETRACT);
    }

    public void stopAngledArm() {
        if (state == ClimbSubsystemState.ENABLED)
            climbDynamicMotor.set(ClimbConstants.STOP_SPEED);
    }

}