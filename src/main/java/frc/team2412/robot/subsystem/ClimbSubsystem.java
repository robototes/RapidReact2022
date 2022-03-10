package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.D;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.ENCODER_TICKS_PER_INCH;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.I;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.MIN_ENCODER_TICKS;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.MOTOR_CURRENT_LIMIT;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.P;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.AutoClimbState;
import frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.SolenoidState;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ClimbSubsystem extends SubsystemBase implements Loggable {

    public static class ClimbConstants {
        public static final double EXTEND_SPEED = 0.1;
        public static final double RETRACT_SPEED = -0.1;
        public static final double STOP_SPEED = 0;

        public static final double ENCODER_TICKS_PER_INCH = 78417 / 11.5 * 2;

        public static final double CLIMB_OFFSET_INCHES = 28.5;

        public static final double MAX_ENCODER_TICKS = (66 - CLIMB_OFFSET_INCHES) * ENCODER_TICKS_PER_INCH; // Max robot
                                                                                                            // height is
                                                                                                            // 66 inches
        public static final double MIN_ENCODER_TICKS = 0;

        public static final int PID_SLOT_0 = 0;
        public static final double P = 0.5;
        public static final double I = 0;
        public static final double D = 0;

        public static final double MID_RUNG_HEIGHT_INCH = 31;
        public static final double HIGH_RUNG_HEIGHT_INCH = 31; // need to test later
        public static final double RETRACT_HEIGHT_INCH = 15;

        public static final SupplyCurrentLimitConfiguration MOTOR_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                true, 40, 60, 15);

        enum HookArmState {
            ANGLED, UPRIGHT
        }

        enum ClimbSubsystemState {
            ENABLED, DISABLED
        }

        public enum AutoClimbState {
            GROUND_MID, MID_HIGH, HIGH_TRAV
        }

        public static enum SolenoidState {
            EXTEND(DoubleSolenoid.Value.kForward, "Extended"), RETRACT(DoubleSolenoid.Value.kReverse, "Reversed");

            public final DoubleSolenoid.Value value;
            public final String state;

            private SolenoidState(DoubleSolenoid.Value value, String state) {
                this.value = value;
                this.state = state;
            }
        }
    }

    @Log.MotorController
    private final WPI_TalonFX climbFixedMotor;

    @Log.MotorController
    private final WPI_TalonFX climbDynamicMotor;

    private DoubleSolenoid solenoid;

    private boolean enabled;
    private static SolenoidState solenoidState = SolenoidState.RETRACT;
    private static AutoClimbState autoClimbState = AutoClimbState.GROUND_MID;

    private double lastUpdatedTime = Timer.getFPGATimestamp();

    public ClimbSubsystem(WPI_TalonFX climbFixedMotor, WPI_TalonFX climbDynamicMotor, DoubleSolenoid climbAngle) {
        setName("ClimbSubsystem");
        this.climbFixedMotor = climbFixedMotor;
        this.climbDynamicMotor = climbDynamicMotor;
        solenoid = climbAngle;

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.forwardSoftLimitEnable = false;
        motorConfig.reverseSoftLimitEnable = false;
        motorConfig.forwardSoftLimitThreshold = MAX_ENCODER_TICKS;
        motorConfig.reverseSoftLimitThreshold = MIN_ENCODER_TICKS;
        motorConfig.supplyCurrLimit = MOTOR_CURRENT_LIMIT;

        climbFixedMotor.configAllSettings(motorConfig);
        climbFixedMotor.setNeutralMode(NeutralMode.Brake);

        setFixedArmPID(P, I, D);

        if (climbDynamicMotor != null) {
            climbDynamicMotor.configAllSettings(motorConfig);
            setDynamicArmPID(P, I, D);
        }

    }

    public void setEnabled() {
        enabled = true;
    }

    public void setAutoClimbState(AutoClimbState newState) {
        autoClimbState = newState;
    }

    public AutoClimbState getAutoClimbState() {
        return autoClimbState;
    }

    public void setDisabled() {
        enabled = false;
    }

    public boolean getState() {
        return enabled;
    }

    public void extendArmSolenoid() {
        solenoid.set(SolenoidState.EXTEND.value);
    }

    public void retractArmSolenoid(){
        solenoid.set(SolenoidState.RETRACT.value);
    }


    @Config(name = "Stop Fixed Motor")
    public void stopFixedPID(boolean stop) {
        if (stop) {
            climbFixedMotor.stopMotor();
        }
    }

    public void extendFixedArm() {
        setMotor(MID_RUNG_HEIGHT_INCH * ENCODER_TICKS_PER_INCH, climbFixedMotor);
    }

    public void retractFixedArm() {
        setMotor(RETRACT_HEIGHT_INCH * ENCODER_TICKS_PER_INCH, climbFixedMotor);
    }

    public void retractFixedArmFully() {
        setMotor(2 * ENCODER_TICKS_PER_INCH, climbFixedMotor);
    }

    public void stopFixedArm() {
        setMotor(ClimbConstants.STOP_SPEED, climbFixedMotor);
    }

    public void extendDynamicArm() {
        setMotor(HIGH_RUNG_HEIGHT_INCH * ENCODER_TICKS_PER_INCH, climbDynamicMotor);
    }

    public void retractDynamicArm() {
        setMotor(RETRACT_HEIGHT_INCH * ENCODER_TICKS_PER_INCH, climbDynamicMotor);
    }

    public void retractDynamicArmFully() {
        setMotor(2 * ENCODER_TICKS_PER_INCH, climbDynamicMotor);
    }

    public void stopAngledArm() {
        setMotor(ClimbConstants.STOP_SPEED, climbDynamicMotor);
    }

    public void setMotor(double value, WPI_TalonFX motor) {
        if (motor != null)
            motor.set(ControlMode.Position, value);
    }

    @Override
    public void simulationPeriodic() {
        // max speed 6000 rpm, 2048 ticks per rotation
        double timeNow = Timer.getFPGATimestamp();
        double timeElapsed = timeNow - lastUpdatedTime;
        double motorFixedSpeed = climbFixedMotor.getSelectedSensorVelocity();
        climbFixedMotor.getSimCollection().setIntegratedSensorRawPosition((int) (motorFixedSpeed / timeElapsed));
        if (climbDynamicMotor != null) {
            double motorDynamicSpeed = climbDynamicMotor.getSelectedSensorVelocity();
            climbDynamicMotor.getSimCollection().setIntegratedSensorRawPosition((int) (motorDynamicSpeed /
                    timeElapsed));
        }

        lastUpdatedTime = timeNow;
    }

    @Log
    public double encoderPosition() {
        return climbFixedMotor.getSelectedSensorPosition();
    }

    @Log
    public boolean isSolenoidExtended() {
        return solenoid.get() == SolenoidState.EXTEND.value;
    }

    @Log
    public double climbHeightInches() {
        return encoderPosition() / ENCODER_TICKS_PER_INCH + CLIMB_OFFSET_INCHES;
    }

    @Config
    public void resetEncoder(boolean reset) {
        if (reset) {
            climbFixedMotor.setSelectedSensorPosition(0);
        }
    }

    @Config(name = "PID")
    private void setFixedArmPID(@Config(name = "P", defaultValueNumeric = P) double p,
            @Config(name = "I", defaultValueNumeric = I) double i,
            @Config(name = "D", defaultValueNumeric = D) double d) {
        climbFixedMotor.config_kP(PID_SLOT_0, p);
        climbFixedMotor.config_kI(PID_SLOT_0, i);
        climbFixedMotor.config_kD(PID_SLOT_0, d);
    }

    @Config(name = "PID")
    private void setDynamicArmPID(@Config(name = "P", defaultValueNumeric = P) double p,
            @Config(name = "I", defaultValueNumeric = I) double i,
            @Config(name = "D", defaultValueNumeric = D) double d) {
        climbDynamicMotor.config_kP(PID_SLOT_0, p);
        climbDynamicMotor.config_kI(PID_SLOT_0, i);
        climbDynamicMotor.config_kD(PID_SLOT_0, d);
    }

}
