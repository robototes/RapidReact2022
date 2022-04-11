package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.IndexSubsystem.IndexConstants.*;
import static frc.team2412.robot.Hardware.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.sim.PhysicsSim;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IndexSubsystem extends SubsystemBase implements Loggable {

    // Constants

    public static class IndexConstants {

        public static final double CURRENT_LIMIT_TRIGGER_SECONDS = 0.5;
        public static final double CURRENT_LIMIT_RESET_AMPS = 10;
        public static final double CURRENT_LIMIT_TRIGGER_AMPS = 20;

        // Index Motor Speeds

        public static final double INDEX_FEEDER_SPEED = 0.125;
        public static final double INDEX_FEEDER_SHOOT_SPEED = 0.3;
        public static final double INDEX_INGEST_SHOOT_SPEED = 0.15;

        public static final double INDEX_IN_SPEED = IntakeSubsystem.IntakeConstants.INTAKE_IN_SPEED / 2;
        public static final double INDEX_OUT_SPEED = -0.3;

        // The current limit
        public static final SupplyCurrentLimitConfiguration MAX_MOTOR_CURRENT = new SupplyCurrentLimitConfiguration(
                true, CURRENT_LIMIT_RESET_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TRIGGER_SECONDS);

    }

    // Define Hardware

    private final DigitalInput leftFeederProximity;
    private final DigitalInput rightFeederProximity;

    @Log.MotorController
    private final WPI_TalonFX ingestMotor;

    @Log.MotorController
    private final WPI_TalonFX feederMotor;

    // Sensor Override

    public boolean ignoreFeeder = false;
    public boolean feederOverridenValue = false; // eddie thinking of new name as we speak 84 ratatouilles a week 🤪

    // Constructor

    public IndexSubsystem() {
        ingestMotor = new WPI_TalonFX(INDEX_INGEST_MOTOR);
        feederMotor = new WPI_TalonFX(INDEX_FEEDER_MOTOR);
        leftFeederProximity = new DigitalInput(LEFT_FEEDER_PROXIMITY);
        rightFeederProximity = new DigitalInput(RIGHT_FEEDER_PROXIMITY);

        ingestMotor.configFactoryDefault();
        feederMotor.configFactoryDefault();

        ingestMotor.setNeutralMode(NeutralMode.Brake);
        feederMotor.setNeutralMode(NeutralMode.Brake);

        ingestMotor.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);
        feederMotor.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);

        feederMotor.setInverted(true);

        ingestMotorStop();
        feederMotorStop();
    }

    // Methods

    public void simInit(PhysicsSim sim) {
        sim.addTalonFX(ingestMotor, 1, SIM_FULL_VELOCITY);
        sim.addTalonFX(feederMotor, 1, SIM_FULL_VELOCITY);
    }

    public void setSpeed(double ingestSpeed, double feederSpeed) {
        System.out.println(ingestSpeed);
        ingestMotor.set(ingestSpeed);
        feederMotor.set(feederSpeed);
    }

    /**
     * Spins first motor inward and updates first motor state
     */
    public void ingestMotorIn() {
        ingestMotor.set(INDEX_IN_SPEED);
    }

    /**
     * Spins first motor outward and updates first motor state
     */
    public void ingestMotorOut() {
        ingestMotor.set(INDEX_OUT_SPEED);
    }

    /**
     * Stops first motor and updates first motor state
     */
    public void ingestMotorStop() {
        ingestMotor.set(0);
    }

    public void ingestMotorShoot() {
        ingestMotor.set(INDEX_INGEST_SHOOT_SPEED);
    }

    public void feederMotorShoot() {
        feederMotor.set(INDEX_FEEDER_SHOOT_SPEED);
    }

    /**
     * Spins second motor inward and updates second motor state
     */
    public void feederMotorIn() {
        feederMotor.set(INDEX_FEEDER_SPEED);
    }

    /**
     * Spins second motor outward and updates second motor state
     */
    public void feederMotorOut() {
        feederMotor.set(INDEX_OUT_SPEED);
    }

    /**
     * Stops second motor and updates second motor state
     */
    public void feederMotorStop() {
        feederMotor.set(0);
    }

    /**
     * Checks if ball is positioned at the sensors
     */
    @Log(name = "Has Cargo")
    public boolean hasCargo() { // might rename methods later?
        if (ignoreFeeder) {
            return feederOverridenValue;
        }
        return (leftFeederProximity.get() && rightFeederProximity.get());
    }

    @Log
    public boolean leftFeederValue() {
        return leftFeederProximity.get();
    }

    @Log
    public boolean rightFeederValue() {
        return rightFeederProximity.get();
    }

    /**
     * Checks if ingest motor is on
     */
    public boolean isIngestMotorOn() {
        return ingestMotor.get() != 0;
    }

    /**
     * Checks if feeder motor is on
     */
    public boolean isFeederMotorOn() {
        return feederMotor.get() != 0;
    }

    private double ingestOverCurrentStart = 0;
    private double feederOverCurrentStart = 0;

    // do need now! :D D: :3 8) B) :P C: xD :p :] E: :} :> .U.
    @Override
    public void periodic() {
        // Checking for jamming
        double ingestCurrent = ingestMotor.getSupplyCurrent();
        if (ingestCurrent > CURRENT_LIMIT_TRIGGER_AMPS) {
            if (ingestOverCurrentStart == 0) {
                ingestOverCurrentStart = Timer.getFPGATimestamp();
            }
        }
        if (ingestCurrent > CURRENT_LIMIT_RESET_AMPS) {
            if (ingestOverCurrentStart > 0) {
                if (Timer.getFPGATimestamp() - ingestOverCurrentStart > CURRENT_LIMIT_TRIGGER_SECONDS) {
                    ingestMotorStop();
                }

            }
        } else {
            ingestOverCurrentStart = 0;
        }

        double feederCurrent = feederMotor.getSupplyCurrent();
        if (feederCurrent > CURRENT_LIMIT_TRIGGER_AMPS) {
            if (feederOverCurrentStart == 0) {
                feederOverCurrentStart = Timer.getFPGATimestamp();
            }
        }
        if (feederCurrent > CURRENT_LIMIT_RESET_AMPS) {
            if (feederOverCurrentStart > 0) {
                if (Timer.getFPGATimestamp() - feederOverCurrentStart > CURRENT_LIMIT_TRIGGER_SECONDS) {
                    feederMotorStop();
                }

            }
        } else {
            feederOverCurrentStart = 0;
        }

    }

    // for logging

    @Log(name = "Ingest Motor Speed")
    public double getIngestMotorSpeed() {
        return ingestMotor.get();
    }

    @Log(name = "Feeder Motor Speed")
    public double getFeederMotorSpeed() {
        return feederMotor.get();
    }

    @Log(name = "Feeder motor moving")
    public boolean isFeederMoving() {
        return isFeederMotorOn();
    }

    @Log(name = "Ingest motor moving")
    public boolean isIngestMoving() {
        return isIngestMotorOn();
    }

    /**
     * sets ignoreIngest value
     *
     * @param ignore
     */
    @Config
    public void setIgnoreFeeder(boolean ignore) {
        ignoreFeeder = ignore;
    }

    /**
     * sets ingestOverridenValue value
     */
    @Config
    public void setFeederOverridenValue(boolean value) {
        feederOverridenValue = value;
    }

}
