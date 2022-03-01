package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.*;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeMotorState.*;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeSolenoidState.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeMotorState;
import frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeSolenoidState;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeSubsystem extends SubsystemBase implements Loggable {

    // Constants

    public static class IntakeConstants {

        public static Alliance teamColor = DriverStation.getAlliance();

        public static final double INTAKE_IN_SPEED = 0.5;
        public static final double INTAKE_OUT_SPEED = -0.5; // will adjust later after testing?

        public static final SupplyCurrentLimitConfiguration MAX_MOTOR_CURRENT = new SupplyCurrentLimitConfiguration(
                true, 40, 40, 500);

        // Enums

        public static enum IntakeMotorState {
            IN, OUT, STOPPED;
        }

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
    private final WPI_TalonFX motorOuterAxle;

    @Log
    private final WPI_TalonFX motorInnerAxle;

    @Log
    private final DoubleSolenoid solenoid;

    // States

    @Log
    public static String state = "";
    private IntakeMotorState intakeMotorState;
    private IntakeSolenoidState intakeSolenoidState;

    // CONSTRUCTOR!

    public IntakeSubsystem(WPI_TalonFX motorOuterAxle,
            WPI_TalonFX motorInnerAxle,
            DoubleSolenoid intakeSolenoid) {

        this.motorOuterAxle = motorOuterAxle;
        this.motorOuterAxle.setInverted(true);
        this.motorInnerAxle = motorInnerAxle;
        this.motorInnerAxle.setNeutralMode(NeutralMode.Coast);
        this.motorOuterAxle.setNeutralMode(NeutralMode.Coast);
        this.motorOuterAxle.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);
        this.motorInnerAxle.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);

        this.solenoid = intakeSolenoid;

        intakeSolenoidState = EXTEND;
        intakeMotorState = STOPPED;

        intakeRetract();
        intakeStop();

        setName("IntakeSubsystem");
    }

    // Methods

    /**
     * Manually sets the speed of the inner and outer axle motors.
     *
     * @param speed
     */
    public void setSpeed(double speed) {
        motorInnerAxle.set(speed);
        motorOuterAxle.set(speed);
    }

    /**
     * Spins motors inwards and updates motor state.
     * Runs if Intake is extended which is when the solenoid is retracted.
     */
    public void intakeIn() {
        if (intakeSolenoidState == RETRACT) {
            motorOuterAxle.set(INTAKE_IN_SPEED);
            motorInnerAxle.set(INTAKE_IN_SPEED);
            intakeMotorState = IN;
        }
    }

    /**
     * Spins motors outwards and updates motor state.
     * Runs if Intake is extended which is when the solenoid is retracted.
     */
    public void intakeOut() {
        if (intakeSolenoidState == RETRACT) {
            motorOuterAxle.set(INTAKE_OUT_SPEED);
            motorInnerAxle.set(INTAKE_OUT_SPEED);
            intakeMotorState = OUT;
        }
    }

    /**
     * Stops motors and updates motor state
     */
    public void intakeStop() {
        motorOuterAxle.stopMotor();
        motorInnerAxle.stopMotor();
        intakeMotorState = STOPPED;
    }

    /**
     * Extends Intake by retract solenoid and updates solenoid state
     */
    public void intakeExtend() {
        intakeSolenoidState = RETRACT;
        solenoid.set(RETRACT.value);
        state = EXTEND.toString();
    }

    /**
     * Retracts Intake by extending solenoid and updates solenoid state
     */
    public void intakeRetract() {
        intakeSolenoidState = EXTEND;
        intakeStop();
        solenoid.set(EXTEND.value);
        state = RETRACT.toString();
    }

    @Override
    public void periodic() {
        if (intakeSolenoidState == RETRACT && intakeMotorState != STOPPED) {
            intakeStop();
        }
    }
}
