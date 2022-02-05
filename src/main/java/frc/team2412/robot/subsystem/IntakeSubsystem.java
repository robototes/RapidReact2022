package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.*;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeMotorState.*;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeSolenoidState.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeMotorState;
import frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeSolenoidState;
import frc.team2412.robot.util.MultiplexedColorSensor;

public class IntakeSubsystem extends SubsystemBase {

    // Constants

    public static class IntakeConstants {

        public static Alliance teamColor = DriverStation.getAlliance();

        public static Color BLUE_CARGO_COLOR = new Color(0, 0, 1);
        public static Color RED_CARGO_COLOR = new Color(1, 0, 0);

        public static final double INTAKE_IN_SPEED = 0.5;
        public static final double INTAKE_OUT_SPEED = -0.5; // will adjust later after testing?

        public static final SupplyCurrentLimitConfiguration MAX_MOTOR_CURRENT = new SupplyCurrentLimitConfiguration(
                true, 40, 40, 500);

        // Enums

        public static enum IntakeMotorState {
            IN, OUT, STOPPED;
        }

        public static enum IntakeSolenoidState {
            EXTEND(DoubleSolenoid.Value.kForward), RETRACT(DoubleSolenoid.Value.kReverse);

            public final DoubleSolenoid.Value value;

            private IntakeSolenoidState(DoubleSolenoid.Value value) {
                this.value = value;
            }
        }
    }

    private ColorMatch allyColorMatcher = new ColorMatch();
    private ColorMatch enemyColorMatcher = new ColorMatch();

    ///// IMPORTANT: Need ball amount variable and make method to stop taking in
    ///// balls when at limit.

    // Define Hardware

    private final WPI_TalonFX motorOuterAxle;
    private final WPI_TalonFX motorInnerAxle;

    private final DoubleSolenoid solenoid;

    // private final ColorSensorV3 colorSensor;

    private final MultiplexedColorSensor leftColorSensor;
    private final MultiplexedColorSensor rightColorSensor;
    private final MultiplexedColorSensor centerColorSensor;

    // States

    private IntakeMotorState intakeMotorState;
    private IntakeSolenoidState intakeSolenoidState;

    // CONSTRUCTOR!

    public IntakeSubsystem(WPI_TalonFX motorOuterAxle,
            WPI_TalonFX motorInnerAxle,
            DoubleSolenoid intakeSolenoid,
            MultiplexedColorSensor leftColorSensor,
            MultiplexedColorSensor rightColorSensor,
            MultiplexedColorSensor centerColorSensor) {

        this.motorOuterAxle = motorOuterAxle;
        this.motorInnerAxle = motorInnerAxle;
        this.motorInnerAxle.setInverted(true);
        this.motorInnerAxle.setNeutralMode(NeutralMode.Coast);
        this.motorOuterAxle.setNeutralMode(NeutralMode.Coast);
        this.motorOuterAxle.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);
        this.motorInnerAxle.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);

        this.solenoid = intakeSolenoid;

        this.leftColorSensor = leftColorSensor;
        this.rightColorSensor = rightColorSensor;
        this.centerColorSensor = centerColorSensor;

        allyColorMatcher.setConfidenceThreshold(0.9);
        enemyColorMatcher.setConfidenceThreshold(0.9);

        // Creates two different color matchers to differentiate between enemy and ally cargo
        if (teamColor == Alliance.Blue) {
            allyColorMatcher.addColorMatch(BLUE_CARGO_COLOR);
            enemyColorMatcher.addColorMatch(RED_CARGO_COLOR);

        } else if (teamColor == Alliance.Red) {
            allyColorMatcher.addColorMatch(RED_CARGO_COLOR);
            enemyColorMatcher.addColorMatch(BLUE_CARGO_COLOR);
        }

        intakeSolenoidState = RETRACT;
        intakeMotorState = STOPPED;

        intakeRetract();
        intakeStop();

        setName("IntakeSubsystem");
    }

    // Methods

    /**
     * Spins motors inwards and updates motor state
     */
    public void intakeIn() {
        if (intakeSolenoidState == EXTEND) {
            motorOuterAxle.set(INTAKE_IN_SPEED);
            motorInnerAxle.set(INTAKE_IN_SPEED);

            intakeMotorState = IN;
        }
    }

    /**
     * Spins motors outwards and updates motor state
     */
    public void intakeOut() {
        if (intakeSolenoidState == IntakeSolenoidState.EXTEND) {
            motorOuterAxle.set(INTAKE_OUT_SPEED);
            motorInnerAxle.set(INTAKE_OUT_SPEED);

            intakeMotorState = OUT;
        }
    }

    /**
     * Stops motors and updates motor state
     */
    public void intakeStop() {
        motorOuterAxle.set(0);
        motorInnerAxle.set(0);

        intakeMotorState = STOPPED;
    }

    /**
     * Extends solenoid and updates solenoid state
     */
    public void intakeExtend() {
        intakeSolenoidState = EXTEND;

        solenoid.set(EXTEND.value);
    }

    /**
     * Retracts solenoid and updates solenoid state
     */
    public void intakeRetract() {
        intakeSolenoidState = RETRACT;
        intakeStop();
        solenoid.set(RETRACT.value);
    }

    /**
     * Returns true if color sensor detects an enemy cargo
     */
    private boolean individualHasOpposingColorCargo(MultiplexedColorSensor colorSensor) {
        return enemyColorMatcher.matchColor(colorSensor.getColor()) != null;
    }

    /**
     * Returns true if any of the color sensors detect an enemy cargo
     */
    public boolean hasOpposingColorCargo() {
        return (individualHasOpposingColorCargo(leftColorSensor)
                || individualHasOpposingColorCargo(rightColorSensor)
                || individualHasOpposingColorCargo(centerColorSensor));
    }

    /**
     * Returns current color value detected if matched
     */
    private boolean individualHasMatchingColorCargo(MultiplexedColorSensor colorSensor) {
        return allyColorMatcher.matchColor(colorSensor.getColor()) != null;
    }

    public boolean hasMatchingColorCargo() {
        return (individualHasMatchingColorCargo(leftColorSensor)
                || individualHasMatchingColorCargo(rightColorSensor)
                || individualHasMatchingColorCargo(centerColorSensor));
    }

    @Override
    public void periodic() {
        if (intakeSolenoidState == RETRACT && intakeMotorState != STOPPED) {
            intakeStop();
        }
    }
}
