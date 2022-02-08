package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.IndexSubsystem.IndexConstants.*;
import static frc.team2412.robot.subsystem.IndexSubsystem.IndexConstants.IndexMotorState.*;
import static frc.team2412.robot.subsystem.IndexSubsystem.IndexConstants.IndexPositionState.*;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.util.MultiplexedColorSensor;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class IndexSubsystem extends SubsystemBase {

    // Constants

    public static class IndexConstants {

        public static Alliance teamColor = DriverStation.getAlliance();

        // Color Sensor values

        public static Color BLUE_CARGO_COLOR = new Color(0, 0, 1);
        public static Color RED_CARGO_COLOR = new Color(1, 0, 0);

        // Index Motor Speeds

        public static final double INDEX_IN_SPEED = 0.5; // will change values later
        public static final double INDEX_OUT_SPEED = -0.5; // will also change later

        // Proximity Threshold
        public static final double PROXIMITY_THRESHOLD = 1000; // value not 1500, to be determined actual
                                                                // value

        // Index Motor States

        public static enum IndexMotorState {
            IN, OUT, STOPPED;
        }

        // Position States

        public static enum IndexPositionState {
            HAS_BALL, HAS_NO_BALL;
        }

        // The current limit
        public static final SupplyCurrentLimitConfiguration MAX_MOTOR_CURRENT = new SupplyCurrentLimitConfiguration(
                true, 40, 40, 500);

    }

    // Define Hardware

    private final MultiplexedColorSensor ingestColorSensor;
    private final MultiplexedColorSensor feederColorSensor;

    private final WPI_TalonFX ingestMotor;
    private final WPI_TalonFX feederMotor;

    // States

    private IndexMotorState ingestMotorState;
    private IndexMotorState feederMotorState;

    private IndexPositionState ingestBallState;
    private IndexPositionState feederBallState;

    // Constructor

    public IndexSubsystem(WPI_TalonFX firstMotor, WPI_TalonFX secondMotor, MultiplexedColorSensor firstColorSensor,
            MultiplexedColorSensor secondColorSensor) {
        this.ingestMotor = firstMotor;
        this.feederMotor = secondMotor;
        this.ingestColorSensor = firstColorSensor;
        this.feederColorSensor = secondColorSensor;

        this.ingestMotor.configFactoryDefault();
        this.feederMotor.configFactoryDefault();

        this.ingestMotor.setNeutralMode(NeutralMode.Coast);
        this.feederMotor.setNeutralMode(NeutralMode.Coast);

        this.ingestMotor.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);
        this.feederMotor.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);

        ingestMotorStop();
        feederMotorStop();

        ingestBallState = HAS_NO_BALL;
        feederBallState = HAS_NO_BALL;

        setName("IndexSubsystem");
    }

    // Methods

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
        if (!ingestSensorHasBallIn()) {
            ingestMotor.set(0);
            ingestMotorState = STOPPED;
        }
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
        return (ingestColorSensor.getProximity() > PROXIMITY_THRESHOLD);
    }

    /**
     * Checks if ball is positioned at the second sensor
     */
    @Log
    public boolean feederSensorHasBallIn() { // might rename methods later?
        return (feederColorSensor.getProximity() > PROXIMITY_THRESHOLD);
    }

    public boolean isIngestMotorOn() {
        return !(ingestMotorState == STOPPED);
    }

    public boolean isFeederMotorOn() {
        return !(feederMotorState == STOPPED);
    }

    // do need now! :D D: :3 8) B) :P C: xD :p :] E: :} :> .U.
    @Override
    public void periodic() {
        if (ingestSensorHasBallIn()) {
            ingestBallState = HAS_BALL;
        } else {
            ingestBallState = HAS_NO_BALL;
        }

        if (feederSensorHasBallIn()) {
            feederBallState = HAS_BALL;
        } else {
            feederBallState = HAS_NO_BALL;
        }
    }

    // for logging

    @Log
    public int getIngestProximity() {
        return ingestColorSensor.getProximity();
    }

    @Log
    public int getFeederProximity() {
        return feederColorSensor.getProximity();
    }

    @Log
    public double getIngestMotorSpeed() {
        return ingestMotor.get();
    }

    @Log
    public double getFeederMotorSpeed() {
        return feederMotor.get();
    }

    // @Log
    // public indexMotorState getIngestMotorState() {
    // return ingestMotorState;
    // }

    // @Log
    // public indexMotorState getFeederMotorState() {
    // return ingestMotorState;
    // }
}
