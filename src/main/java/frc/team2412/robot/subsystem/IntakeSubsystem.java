package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.*;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeMotorState.*;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeSolenoidState.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;
import frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeMotorState;
import frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeSolenoidState;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeSubsystem extends SubsystemBase implements Loggable {

    // Constants
    public static class IntakeConstants {

        public static final double INTAKE_IN_SPEED = 0.7;
        public static final double INTAKE_OUT_SPEED = -0.7; // will adjust later after testing?

        public static final SupplyCurrentLimitConfiguration MAX_MOTOR_CURRENT = new SupplyCurrentLimitConfiguration(
                true, 20, 20, 1);

        // Enums

        public static enum IntakeMotorState {
                                                IN, OUT, STOPPED;
        }

        public static enum IntakeSolenoidState {
                                                EXTEND(DoubleSolenoid.Value.kForward, "Extended"),
                                                RETRACT(DoubleSolenoid.Value.kReverse, "Reversed");

            public final DoubleSolenoid.Value value;
            public final String state;

            private IntakeSolenoidState(DoubleSolenoid.Value value, String state) {
                this.value = value;
                this.state = state;
            }
        }
    }

    // Define Hardware

    @Log
    private final WPI_TalonFX motor1;
    private final WPI_TalonFX motor2;

    // @Log
    private final DoubleSolenoid solenoid;

    // States

    @Log(name = "Solenoid State")
    public static String state = "";
    private IntakeMotorState intakeMotorState;
    private IntakeSolenoidState intakeSolenoidState;

    // CONSTRUCTOR!

    private IntakeSubsystem() {
        var hardware = Hardware.instance;

        this.motor1 = hardware.intakeMotor;
        this.motor1.setNeutralMode(NeutralMode.Coast);
        this.motor1.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);
        this.motor2 = hardware.intakeMotor2;

        if (this.motor2 != null) {
            this.motor2.setNeutralMode(NeutralMode.Coast);
            this.motor2.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);
        }

        this.solenoid = hardware.intakeSolenoid;

        intakeSolenoidState = EXTEND;
        intakeMotorState = STOPPED;

        intakeRetract();
        intakeStop();

        setName("IntakeSubsystem");
    }

    // Methods

    /**
     * Manually sets the speed of the motor
     *
     * @param speed
     */
    public void setSpeed(double speed) {
        motor1.set(speed);
        if (motor2 != null)
            motor2.set(-speed);
    }

    /**
     * Spins motor inwards and updates motor state. Runs if Intake is extended which is when the solenoid is
     * retracted.
     */
    public void intakeIn() {
        if (isIntakeExtended()) {
            setSpeed(INTAKE_IN_SPEED);
            intakeMotorState = IN;
        }
    }

    /**
     * Spins motor outwards and updates motor state. Runs if Intake is extended which is when the solenoid is
     * retracted.
     */
    public void intakeOut() {
        if (isIntakeExtended()) {
            setSpeed(INTAKE_OUT_SPEED);
            intakeMotorState = OUT;
        }
    }

    /**
     * Stops motor and updates motor state
     */
    public void intakeStop() {
        setSpeed(0);
        intakeMotorState = STOPPED;
    }

    /**
     * Extends Intake by retract solenoid and updates solenoid state
     */
    public void intakeExtend() {
        intakeSolenoidState = RETRACT;
        if (solenoid != null)
            solenoid.set(RETRACT.value);
        state = EXTEND.toString();
    }

    /**
     * Retracts Intake by extending solenoid and updates solenoid state
     */
    public void intakeRetract() {
        intakeSolenoidState = EXTEND;
        intakeStop();
        if (solenoid != null)
            solenoid.set(EXTEND.value);
        state = RETRACT.toString();
    }

    @Override
    public void periodic() {
        // if (intakeSolenoidState == RETRACT && intakeMotorState != STOPPED) {
        // intakeStop();
        // }
    }

    // Logging Methods

    /**
     * Gets the motor speed
     */
    @Log(name = "Motor Speed")
    public double getMotorSpeed() {
        return motor1.get();
    }

    /**
     * Checks if motor is on
     */
    @Log(name = "Motor Moving")
    public boolean isMotorOn() {
        return motor1.get() != 0;
    }

    /**
     * Checks if Intake is extended
     */
    @Log(name = "Intake Extended")
    public boolean isIntakeExtended() {
        return (intakeSolenoidState == RETRACT);
    }

    // Singleton
    public static final IntakeSubsystem instance = new IntakeSubsystem();
}
