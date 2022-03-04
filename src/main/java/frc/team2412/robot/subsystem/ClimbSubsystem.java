package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.ARM_REACH_DISTANCE;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.D;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.ENCODER_TICKS_PER_INCH;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.ENCODER_TICKS_PER_REVOLUTION;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.GEARBOX_REDUCTION;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.I;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.MIN_ENCODER_TICKS;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.MOTOR_CURRENT_LIMIT;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.P;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.RUNG_DISTANCE;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.TEST_SPEED_EXTEND;
import static frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.AutoClimbState;
import frc.team2412.robot.subsystem.ClimbSubsystem.ClimbConstants.SolenoidState;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ClimbSubsystem extends SubsystemBase implements Loggable {

    public static class ClimbConstants {
        public static final double MAX_SPEED = 1;
        public static final double TEST_SPEED_EXTEND = 0.1;
        public static final double TEST_SPEED_RETRACT = -0.1;
        public static final double STOP_SPEED = 0;
        public static final double MAX_ENCODER_TICKS = 1000;
        public static final double MIN_ENCODER_TICKS = 0.8;
        public static final double RUNG_DISTANCE = 24; // inches
        public static final double GEARBOX_REDUCTION = 10.61;
        public static final double ENCODER_TICKS_PER_REVOLUTION = 2048;
        public static final double ARM_REACH_DISTANCE = Math.PI * RUNG_DISTANCE * GEARBOX_REDUCTION
                * ENCODER_TICKS_PER_REVOLUTION;

        public static final double ENCODER_TICKS_PER_INCH = 78417 / 11.5;

        public static final double P = 0.5;
        public static final double I = 0;
        public static final double D = 0;

        public static final int PID_SLOT_0 = 0;

        public static final double MID_RUNG_HEIGHT_INCH = 30;
        public static final double RETRACT_HEIGHT_INCH = 2;

        public static final double CLIMB_OFFSET_INCHES = 28.5;

        public static final SupplyCurrentLimitConfiguration MOTOR_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                true, 40, 40, 500);

        enum HookArmState {
            ANGLED, UPRIGHT
        }

        enum ClimbSubsystemState {
            ENABLED, DISABLED
        }

        public enum AutoClimbState {
            GROUND_MID, MID_HIGH, HIGH_TRAV
        }

        enum SolenoidState {
            BACK, MID, FRONT
        }
    }

    @Log.MotorController
    private final WPI_TalonFX climbFixedMotor;

    @Log.MotorController
    private final WPI_TalonFX climbDynamicMotor;

    private DoubleSolenoid solenoid;

    private boolean enabled;
    private static SolenoidState solenoidState = SolenoidState.MID;
    private static AutoClimbState autoClimbState = AutoClimbState.GROUND_MID;

    private final NetworkTableEntry testSpeedExtend;
    private final NetworkTableEntry testSpeedRetract;
    private final NetworkTableEntry maxEncoderTicks;
    private final NetworkTableEntry minEncoderTicks;
    private final NetworkTableEntry rungDistance;
    private final NetworkTableEntry gearboxReduction;
    private final NetworkTableEntry encoderTicksPerRevolution;

    private double lastUpdatedTime = Timer.getFPGATimestamp();

    public ClimbSubsystem(WPI_TalonFX climbFixedMotor, WPI_TalonFX climbDynamicMotor, DoubleSolenoid climbAngle) {
        setName("ClimbSubsystem");
        this.climbFixedMotor = climbFixedMotor;
        this.climbDynamicMotor = climbDynamicMotor;
        // solenoid = climbAngle;

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.forwardSoftLimitEnable = false;
        motorConfig.reverseSoftLimitEnable = false;
        motorConfig.forwardSoftLimitThreshold = ARM_REACH_DISTANCE;
        motorConfig.reverseSoftLimitThreshold = MIN_ENCODER_TICKS;
        motorConfig.supplyCurrLimit = MOTOR_CURRENT_LIMIT;

        climbFixedMotor.configAllSettings(motorConfig);
        climbFixedMotor.setNeutralMode(NeutralMode.Brake);

        climbFixedMotor.config_kP(PID_SLOT_0, P);

        climbDynamicMotor.configAllSettings(motorConfig);

        ShuffleboardTab tab = Shuffleboard.getTab("Climb");

        maxEncoderTicks = tab.add("Max encoder ticks", ARM_REACH_DISTANCE)
                .withPosition(0, 0)
                .withSize(2, 1)
                .getEntry();
        minEncoderTicks = tab.add("Min encoder ticks", MIN_ENCODER_TICKS)
                .withPosition(0, 1)
                .withSize(2, 1)
                .getEntry();
        encoderTicksPerRevolution = tab.add("Encoder ticks per revolution", ENCODER_TICKS_PER_REVOLUTION)
                .withPosition(0, 2)
                .withSize(2, 1)
                .getEntry();
        testSpeedExtend = tab.add("Test speed: Extension", TEST_SPEED_EXTEND)
                .withPosition(2, 0)
                .withSize(2, 1)
                .getEntry();
        testSpeedRetract = tab.add("Test speed: Retraction", TEST_SPEED_RETRACT)
                .withPosition(2, 1)
                .withSize(2, 1)
                .getEntry();
        rungDistance = tab.add("Rung distance", RUNG_DISTANCE)
                .withPosition(4, 0)
                .withSize(1, 2)
                .getEntry();
        gearboxReduction = tab.add("Gearbox reduction", GEARBOX_REDUCTION)
                .withPosition(2, 2)
                .withSize(2, 1)
                .getEntry();
        tab.addNumber("Solenoid state", () -> {
            return solenoidState == SolenoidState.BACK ? -1 : solenoidState == SolenoidState.MID ? 0 : 1;
        })
                .withPosition(4, 4)
                .withSize(2, 1);
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

    public void angleClimbHook(DoubleSolenoid.Value value) {
        solenoid.set(value);
    }

    public void extendFixedArm() {
        setMotor(MID_RUNG_HEIGHT_INCH * ENCODER_TICKS_PER_INCH, climbFixedMotor);
    }

    public void retractFixedArm() {
        setMotor(RETRACT_HEIGHT_INCH * ENCODER_TICKS_PER_INCH, climbFixedMotor);
    }

    public void stopFixedArm() {
        setMotor(ClimbConstants.STOP_SPEED, climbFixedMotor);
    }

    public void extendAngledArm() {
        setMotor(testSpeedExtend.getValue().getDouble(), climbDynamicMotor);
    }

    public void retractAngledArm() {
        setMotor(testSpeedRetract.getValue().getDouble(), climbDynamicMotor);
    }

    public void stopAngledArm() {
        setMotor(ClimbConstants.STOP_SPEED, climbDynamicMotor);
    }

    public void setMotor(double value, WPI_TalonFX motor) {
        // if (enabled) {
        // System.out.println("motor set: " + value);
        // if (Robot.isSimulation()) {
        // motor.getSimCollection().setIntegratedSensorVelocity((int) value);
        // } else {
        motor.set(ControlMode.Position, value);
    }

    public boolean isFixedFullyExtended() {
        return climbFixedMotor.getSelectedSensorPosition() >= maxEncoderTicks.getValue().getDouble();
    }

    public boolean isFixedFullyRetracted() {
        return climbFixedMotor.getSelectedSensorPosition() <= minEncoderTicks.getValue().getDouble();
    }

    public boolean isDynamicFullyExtended() {
        return climbDynamicMotor.getSelectedSensorPosition() >= maxEncoderTicks.getValue().getDouble();
    }

    public boolean isDynamicFullyRetracted() {
        return climbDynamicMotor.getSelectedSensorPosition() <= minEncoderTicks.getValue().getDouble();
    }

    @Override
    public void simulationPeriodic() {
        // max speed 6000 rpm, 2048 ticks per rotation
        double timeNow = Timer.getFPGATimestamp();
        double timeElapsed = timeNow - lastUpdatedTime;
        double motorFixedSpeed = climbFixedMotor.getSelectedSensorVelocity();
        double motorDynamicSpeed = climbDynamicMotor.getSelectedSensorVelocity();
        climbFixedMotor.getSimCollection().setIntegratedSensorRawPosition((int) (motorFixedSpeed / timeElapsed));
        climbDynamicMotor.getSimCollection().setIntegratedSensorRawPosition((int) (motorDynamicSpeed / timeElapsed));
        lastUpdatedTime = timeNow;
    }

    @Log
    public double encoderPosition() {
        return climbFixedMotor.getSelectedSensorPosition();
    }

    @Log
    public double climbHeightInches(){
        return encoderPosition() / ENCODER_TICKS_PER_INCH  + CLIMB_OFFSET_INCHES;
    }

    @Config
    public void resetEncoder(boolean reset) {
        if (reset) {
            climbFixedMotor.setSelectedSensorPosition(0);
        }
    }

    @Config(name = "PID")
    private void setPID(@Config(name = "P", defaultValueNumeric = P) double p,
            @Config(name = "I", defaultValueNumeric = I) double i,
            @Config(name = "D", defaultValueNumeric = D) double d) {
        climbFixedMotor.config_kP(PID_SLOT_0, p);
        climbFixedMotor.config_kI(PID_SLOT_0, i);
        climbFixedMotor.config_kD(PID_SLOT_0, d);
    }

    @Config(name = "Climb to Height")
    public void fixedClimbToHeight(double heightInches){
        if(heightInches < 28.5) heightInches = 28.5;
        climbFixedMotor.set(ControlMode.Position, (heightInches - CLIMB_OFFSET_INCHES) * ENCODER_TICKS_PER_INCH);
    }

}
