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

import com.revrobotics.ColorMatch;

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

    // New ColorMatching Instance

    private ColorMatch colorMatcher = new ColorMatch();

    // Define Hardware

    private final MultiplexedColorSensor firstColorSensor;
    private final MultiplexedColorSensor secondColorSensor;

    private final WPI_TalonFX firstMotor;
    private final WPI_TalonFX secondMotor;

    // States

    private IndexMotorState firstMotorState;
    private IndexMotorState secondMotorState;

    private IndexPositionState firstBallState;
    private IndexPositionState secondBallState;

    // Constructor

    public IndexSubsystem(WPI_TalonFX firstMotor, WPI_TalonFX secondMotor, MultiplexedColorSensor firstColorSensor,
            MultiplexedColorSensor secondColorSensor) {
        this.firstMotor = firstMotor;
        this.secondMotor = secondMotor;
        this.firstColorSensor = firstColorSensor;
        this.secondColorSensor = secondColorSensor;

        this.firstMotor.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);
        this.secondMotor.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);

        colorMatcher.addColorMatch(BLUE_CARGO_COLOR); // might change later to account for opposing team color
        colorMatcher.addColorMatch(RED_CARGO_COLOR);

        colorMatcher.setConfidenceThreshold(0.9);

        firstMotorStop();
        secondMotorStop();

        firstBallState = HAS_NO_BALL;
        secondBallState = HAS_NO_BALL;

        setName("IndexSubsystem");
    }

    // Methods

    /**
     * Spins first motor inward and updates first motor state
     */
    public void firstMotorIn() {
        firstMotor.set(INDEX_IN_SPEED);
        firstMotorState = IN;
    }

    /**
     * Spins first motor outward and updates first motor state
     */
    public void firstMotorOut() {
        firstMotor.set(INDEX_OUT_SPEED);
        firstMotorState = OUT;
    }

    /**
     * Stops first motor and updates first motor state
     */
    public void firstMotorStop() {
        if (!firstSensorHasBallIn()) {
            firstMotor.set(0);
            firstMotorState = STOPPED;
        }
    }

    /**
     * Spins second motor inward and updates second motor state
     */
    public void secondMotorIn() {
        secondMotor.set(INDEX_IN_SPEED);
        secondMotorState = IN;
    }

    /**
     * Spins second motor outward and updates second motor state
     */
    public void secondMotorOut() {
        secondMotor.set(INDEX_OUT_SPEED);
        secondMotorState = OUT;
    }

    /**
     * Stops second motor and updates second motor state
     */
    public void secondMotorStop() {
        secondMotor.set(0);
        secondMotorState = STOPPED;
    }

    /**
     * Checks if ball is positioned at the first sensor
     */
    public boolean firstSensorHasBallIn() { // also might rename later?
        return (firstColorSensor.getProximity() > 1500); // value not 1500, to be determined actual value
    }

    /**
     * Checks if ball is positioned at the second sensor
     */
    public boolean secondSensorHasBallIn() { // might rename methods later?
        return (secondColorSensor.getProximity() > 1500); // value not 1500, to be determined actual value
    }

    public boolean isFirstMotorOn() {
        return !(firstMotorState == STOPPED);
    }

    public boolean isSecondMotorOn() {
        return !(secondMotorState == STOPPED);
    }

    // do need now! :D D: :3 8) B) :P C: xD :p :] E: :} 
    @Override
    public void periodic() {
        if (firstSensorHasBallIn()) {
            firstBallState = HAS_BALL;
        } else {
            firstBallState = HAS_NO_BALL;
        }

        if (secondSensorHasBallIn()) {
            secondBallState = HAS_BALL;
        } else {
            secondBallState = HAS_NO_BALL;
        }
    }

}
