package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.*;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeSolenoidState.*;
import static frc.team2412.robot.Hardware.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;
import frc.team2412.robot.sim.PhysicsSim;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeSubsystem extends SubsystemBase implements Loggable {

    // Constants
    public static class IntakeConstants {

        public static final double INNER_INTAKE_IN_SPEED = 0.35; // TODO
        public static final double INTAKE_IN_SPEED = 0.7;
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

    // Define Hardware

    @Log
    private final WPI_TalonFX motor1;
    private final WPI_TalonFX motor2;

    private final DoubleSolenoid solenoid;

    private final DigitalInput ingestProximity;

    // States

    @Log(name = "Solenoid State")
    public static String state = "";

    public boolean ignoreIngest = false;
    public boolean ingestOverridenValue = false; // eddie thinking of new name as we speak 84 ratatouilles a week 🤪

    // CONSTRUCTOR!

    public IntakeSubsystem() {

        motor1 = new WPI_TalonFX(INTAKE_MOTOR_1, Hardware.DRIVETRAIN_INTAKE_CAN_BUS_NAME);

        motor1.setNeutralMode(NeutralMode.Coast);
        motor1.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);
        motor1.setInverted(true);
        motor2 = new WPI_TalonFX(INTAKE_MOTOR_2, Hardware.DRIVETRAIN_INTAKE_CAN_BUS_NAME);

        if (motor2 != null) {
            motor2.setNeutralMode(NeutralMode.Coast);
            motor2.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);
            motor2.setInverted(true);
        }

        solenoid = new DoubleSolenoid(PNEUMATIC_HUB, PneumaticsModuleType.REVPH, INTAKE_SOLENOID_UP,
                INTAKE_SOLENOID_DOWN);

        ingestProximity = new DigitalInput(INGEST_PROXIMITY);

        intakeRetract();
        intakeStop();

        setName("IntakeSubsystem");
    }

    // Methods

    public void simInit(PhysicsSim sim) {
        sim.addTalonFX(motor1, 1, SIM_FULL_VELOCITY);
        sim.addTalonFX(motor2, 1, SIM_FULL_VELOCITY);
    }

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
     * Spins motor inwards and updates motor state.
     * Runs if Intake is extended which is when the solenoid is retracted.
     */
    public void intakeIn() {
        if (isIntakeExtended()) {
            setSpeed(INTAKE_IN_SPEED);
        }
    }

    /**
     * Spins motor outwards and updates motor state.
     * Runs if Intake is extended which is when the solenoid is retracted.
     */
    public void intakeOut() {
        if (isIntakeExtended()) {
            setSpeed(INTAKE_OUT_SPEED);
        }
    }

    /**
     * Stops motor and updates motor state
     */
    public void intakeStop() {
        motor1.stopMotor();
        motor2.stopMotor();
    }

    /**
     * Extends Intake by retract solenoid
     */
    public void intakeExtend() {
        if (solenoid != null)
            solenoid.set(RETRACT.value);
        state = EXTEND.toString();
    }

    /**
     * Retracts Intake by extending solenoid
     */
    public void intakeRetract() {
        intakeStop();
        if (solenoid != null)
            solenoid.set(EXTEND.value);
        state = RETRACT.toString();
    }

    @Override
    public void periodic() {
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
        return solenoid.get() == RETRACT.value;
    }

    @Log(name = "solenoid state")
    public String solenoidState() {
        return solenoid.get().toString();
    }

    /**
     * Checks if sensor is detecting ball
     */
    @Log
    public boolean hasCargo() {

        if (ignoreIngest) {
            return ingestOverridenValue;
        }
        return ingestProximity.get();
    }

    /**
     * sets ignoreIngest value
     *
     * @param ignore
     */
    @Config
    public void setIgnoreIngest(boolean ignore) {
        ignoreIngest = ignore;
    }

    /**
     * sets ingestOverridenValue value
     */
    @Config
    public void setIngestOverridenValue(boolean value) {
        ingestOverridenValue = value;
    }
}
