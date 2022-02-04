package frc.team2412.robot.subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;

import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {

    // Constants

    public static class IntakeConstants {
        public static Alliance teamColor = DriverStation.getAlliance();

        public static final double INTAKE_IN_SPEED = 0.5;
        public static final double INTAKE_OUT_SPEED = -0.5;

        public static final SupplyCurrentLimitConfiguration MAX_MOTOR_CURRENT = new SupplyCurrentLimitConfiguration(
                true, 40, 40, 500);

    }

    // Enums

    public enum IntakeMotorState {
        IN, OUT, STOPPED;
    }

    public enum IntakeSolenoidState {
        EXTENDED, RETRACTED;
    }

    public enum IntakeState {
        EXTEND(DoubleSolenoid.Value.kForward), RETRACT(DoubleSolenoid.Value.kReverse);

        public DoubleSolenoid.Value value;

        private IntakeState(DoubleSolenoid.Value value) {
            this.value = value;
        }
    }

    ///// IMPORTANT: Need ball amount variable and make method to stop taking in
    ///// balls when at limit.

    // Define Hardware

    private final WPI_TalonFX motorOuterAxle;

    private final WPI_TalonFX motorInnerAxle;

    private final DoubleSolenoid solenoid;

    /// private final ColorSensorV3 colorSensor = new colorSensorV3(i2cPort);

    private IntakeMotorState intakeMotorState;

    private IntakeSolenoidState intakeSolenoidState;

    // CONSTRUCTOR!
    public IntakeSubsystem(WPI_TalonFX motorOuterAxle, WPI_TalonFX motorInnerAxle, DoubleSolenoid intakeSolenoid) {
        this.motorOuterAxle = motorOuterAxle;
        this.motorInnerAxle = motorInnerAxle;
        this.motorInnerAxle.setInverted(true);

        this.motorInnerAxle.setNeutralMode(NeutralMode.Coast);
        this.motorOuterAxle.setNeutralMode(NeutralMode.Coast);

        this.motorOuterAxle.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);
        this.motorInnerAxle.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);

        this.solenoid = intakeSolenoid;
        // this.colorSensor = intakeColorSensor;
        intakeSolenoidState = IntakeSolenoidState.RETRACTED;
        intakeMotorState = IntakeMotorState.STOPPED;

        intakeRetract();
        intakeStop();

        setName("IntakeSubsystem");
    }

    // Methods.

    public void intakeIn() {
        if (intakeSolenoidState == IntakeSolenoidState.EXTENDED) {
            motorOuterAxle.set(IntakeConstants.INTAKE_IN_SPEED);
            motorInnerAxle.set(IntakeConstants.INTAKE_IN_SPEED);

            intakeMotorState = IntakeMotorState.IN;
        }
    }

    public void intakeOut() {
        if (intakeSolenoidState == IntakeSolenoidState.EXTENDED) {
            motorOuterAxle.set(IntakeConstants.INTAKE_OUT_SPEED);
            motorInnerAxle.set(IntakeConstants.INTAKE_OUT_SPEED);

            intakeMotorState = IntakeMotorState.OUT;
        }
    }

    public void intakeStop() {
        motorOuterAxle.set(0);
        motorInnerAxle.set(0);

        intakeMotorState = IntakeMotorState.STOPPED;
    }

    public void intakeExtend() {
        intakeSolenoidState = IntakeSolenoidState.EXTENDED;

        solenoid.set(IntakeState.EXTEND.value);
    }

    public void intakeRetract() {
        intakeSolenoidState = IntakeSolenoidState.RETRACTED;

        solenoid.set(IntakeState.RETRACT.value);
    }

    public boolean colorIdentify() {
        return true;
    }

    @Override
    public void periodic() {
        intakeSolenoidState = IntakeSolenoidState.EXTENDED;
        Joystick joystick = new Joystick(0);
        solenoid.set(joystick.getTwist() != 0 ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        motorInnerAxle.set(joystick.getX());
        motorOuterAxle.set(joystick.getY());
    }

}
