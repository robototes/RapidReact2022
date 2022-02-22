package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.IndexSubsystem.IndexConstants.*;
import static frc.team2412.robot.subsystem.IndexSubsystem.IndexConstants.IndexMotorState.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import io.github.oblarg.oblog.annotations.Log;

public class IndexSubsystem extends SubsystemBase {

    // Constants

    public static class IndexConstants {

        public static Alliance teamColor = DriverStation.getAlliance();

        public static double CURRENT_LIMIT_TRIGGER_SECONDS = 5;
        public static double CURRENT_LIMIT_RESET_AMPS = 10;
        public static double CURRENT_LIMIT_TRIGGER_AMPS = 20;

        // Index Motor Speeds

        public static double INDEX_IN_SPEED = 0.5; // will change values later
        public static double INDEX_OUT_SPEED = -0.5; // will also change later

        // Proximity Threshold
        public static double PROXIMITY_THRESHOLD = 700; // value not 700, to be determined actual
                                                        // value

        // Index Motor States

        public static enum IndexMotorState {
            IN, OUT, STOPPED;
        }

        // The current limit
        public static final SupplyCurrentLimitConfiguration MAX_MOTOR_CURRENT = new SupplyCurrentLimitConfiguration(
                true, 40, 40, 500);

    }

    // Define Hardware

    private final DigitalInput ingestProximity;
    private final DigitalInput feederProximity;

    private final DigitalInput ingestBlueColor;
    private final DigitalInput ingestRedColor;
    private final DigitalInput feederBlueColor;
    private final DigitalInput feederRedColor;

    private final WPI_TalonFX ingestMotor;
    private final WPI_TalonFX feederMotor;

    // States
    double ingestOverCurrentStart = 0;
    double feederOverCurrentStart = 0;
    private IndexMotorState ingestMotorState;
    private IndexMotorState feederMotorState;

    private boolean ingestBallState;
    private boolean feederBallState;

    // For logging
    private final NetworkTableEntry proximityThreshold;

    // Constructor

    public IndexSubsystem(WPI_TalonFX firstMotor, WPI_TalonFX secondMotor, DigitalInput ingestProximity,
            DigitalInput feederProximity, DigitalInput ingestBlueColor, DigitalInput ingestRedColor,
            DigitalInput feederBlueColor, DigitalInput feederRedColor) {

        ShuffleboardTab tab = Shuffleboard.getTab("Index");

        proximityThreshold = tab.add("Proximity Threshold", PROXIMITY_THRESHOLD)
                .withPosition(0, 0)
                .withSize(2, 1)
                .getEntry();

        tab.addBoolean("ingest sensor proximity", this::getIngestProximity);
        tab.addBoolean("feeder sensor proximity", this::getFeederProximity);
        tab.addNumber("ingest motor speed", this::getIngestMotorSpeed);
        tab.addNumber("feeder motor speed", this::getFeederMotorSpeed);

        tab.addBoolean("ingest sensor has ball", this::ingestSensorHasBallIn);
        tab.addBoolean("feeder sensor has ball", this::feederSensorHasBallIn);
        tab.addBoolean("feeder is moving", this::isFeederMoving);
        tab.addBoolean("feeder is not moving", this::isFeederStopped);

        this.ingestMotor = firstMotor;
        this.feederMotor = secondMotor;
        this.ingestProximity = ingestProximity;
        this.feederProximity = feederProximity;
        this.ingestBlueColor = ingestBlueColor;
        this.ingestRedColor = ingestRedColor;
        this.feederBlueColor = feederBlueColor;
        this.feederRedColor = feederRedColor;

        this.ingestMotor.configFactoryDefault();
        this.feederMotor.configFactoryDefault();

        this.ingestMotor.setNeutralMode(NeutralMode.Brake);
        this.feederMotor.setNeutralMode(NeutralMode.Brake);

        this.ingestMotor.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);
        this.feederMotor.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);

        ingestMotorStop();
        feederMotorStop();

        ingestBallState = false;
        feederBallState = false;

        setName("IndexSubsystem");
    }

    // Methods

    public void setSpeed(double ingestSpeed, double feederSpeed) {
        ingestMotor.set(ingestSpeed);
        feederMotor.set(feederSpeed);
    }

    /**
     * Spins first motor inward and updates first motor state
     */
    public void ingestMotorIn() {
        ingestMotor.set(INDEX_IN_SPEED);
        ingestMotorState = IN;
    }

    /**
     * Spins first motor outward and updates first motor state
     */
    public void ingestMotorOut() {
        ingestMotor.set(INDEX_OUT_SPEED);
        ingestMotorState = OUT;
    }

    /**
     * Stops first motor and updates first motor state
     */
    public void ingestMotorStop() {
        ingestMotor.set(0);
        ingestMotorState = STOPPED;
    }

    /**
     * Spins second motor inward and updates second motor state
     */
    public void feederMotorIn() {
        feederMotor.set(INDEX_IN_SPEED);
        feederMotorState = IN;
    }

    /**
     * Spins second motor outward and updates second motor state
     */
    public void feederMotorOut() {
        feederMotor.set(INDEX_OUT_SPEED);
        feederMotorState = OUT;
    }

    /**
     * Stops second motor and updates second motor state
     */
    public void feederMotorStop() {
        feederMotor.set(0);
        feederMotorState = STOPPED;
    }

    /**
     * Checks if ball is positioned at the first sensor
     */
    @Log
    public boolean ingestSensorHasBallIn() { // also might rename later?
        return ingestProximity.get();
    }

    /**
     * Checks if ball is positioned at the second sensor
     */
    @Log
    public boolean feederSensorHasBallIn() { // might rename methods later?
        return feederProximity.get();
    }

    /**
     * Checks if ingest motor is on
     */
    public boolean isIngestMotorOn() {
        return !(ingestMotorState == STOPPED);
    }

    /**
     * Checks if feeder motor is on
     */
    public boolean isFeederMotorOn() {
        return !(feederMotorState == STOPPED);
    }

    /**
     * Checks if ingest has the correct cargo
     */
    @Log
    public boolean ingestHasCorrectCargo() {
        return ((teamColor == Alliance.Blue && ingestBlueColor.get())
                || teamColor == Alliance.Red && ingestRedColor.get());
    }

    /**
     * Checks if feeder has the correct cargo
     */
    @Log
    public boolean feederHasCorrectCargo() {
        return ((teamColor == Alliance.Blue && feederBlueColor.get())
                || teamColor == Alliance.Red && feederRedColor.get());
    }

    // do need now! :D D: :3 8) B) :P C: xD :p :] E: :} :> .U.
    @Override
    public void periodic() {
        ingestBallState = ingestSensorHasBallIn();
        feederBallState = feederSensorHasBallIn();

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

    public boolean getIngestProximity() {
        return ingestProximity.get();
    }

    public boolean getFeederProximity() {
        return feederProximity.get();
    }

    public double getIngestMotorSpeed() {
        return ingestMotor.get();
    }

    public double getFeederMotorSpeed() {
        return feederMotor.get();
    }

    public boolean isFeederMoving() {
        return feederMotorState != STOPPED;
    }

    public boolean isIngestMoving() {
        return ingestMotorState != STOPPED;
    }

    public boolean isFeederStopped() {
        return feederMotorState == STOPPED;
    }
}
