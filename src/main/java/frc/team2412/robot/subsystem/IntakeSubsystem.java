package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.BLUE_CARGO_COLOR;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.MAX_MOTOR_CURRENT;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.RED_CARGO_COLOR;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.teamColor;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeMotorState.STOPPED;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeSolenoidState.EXTEND;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeSolenoidState.RETRACT;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeMotorState.IN;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeMotorState.OUT;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.IntakeMotorState.STOPPED;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.INTAKE_OUT_SPEED;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.INTAKE_IN_SPEED;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

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
        public static final double INTAKE_OUT_SPEED = -0.5;

        public static final SupplyCurrentLimitConfiguration MAX_MOTOR_CURRENT = new SupplyCurrentLimitConfiguration(
                true, 40, 40, 500);

        // Enums

        public static enum IntakeMotorState {
            IN, OUT, STOPPED;
        }

        public static enum IntakeSolenoidState {
            EXTEND(DoubleSolenoid.Value.kForward), RETRACT(DoubleSolenoid.Value.kReverse);

            public DoubleSolenoid.Value value;

            private IntakeSolenoidState(DoubleSolenoid.Value value) {
                this.value = value;
            }
        }

        // public enum IntakeSolenoidState {
        //     EXTENDED, RETRACTED;
        // }
    }

    private ColorMatch colorMatcher = new ColorMatch();
    
    ///// IMPORTANT: Need ball amount variable and make method to stop taking in
    ///// balls when at limit.

    // Define Hardware

    private final WPI_TalonFX motorOuterAxle;
    private final WPI_TalonFX motorInnerAxle;

    private final DoubleSolenoid solenoid;

    private final ColorSensorV3 colorSensor;

    // private final MultiplexedColorSensor leftColorSensor;
    // private final MultiplexedColorSensor rightColorSensor;
    // private final MultiplexedColorSensor centerColorSensor;

    // States

    private IntakeMotorState intakeMotorState;
    private IntakeSolenoidState intakeSolenoidState;

    // CONSTRUCTOR!

    public IntakeSubsystem(WPI_TalonFX motorOuterAxle, WPI_TalonFX motorInnerAxle, DoubleSolenoid intakeSolenoid, ColorSensorV3 colorSensor) {
        
        this.motorOuterAxle = motorOuterAxle;
        this.motorInnerAxle = motorInnerAxle;
        this.motorInnerAxle.setInverted(true);
        this.motorInnerAxle.setNeutralMode(NeutralMode.Coast);
        this.motorOuterAxle.setNeutralMode(NeutralMode.Coast);
        this.motorOuterAxle.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);
        this.motorInnerAxle.configSupplyCurrentLimit(MAX_MOTOR_CURRENT);

        this.solenoid = intakeSolenoid;

        this.colorSensor = colorSensor;

        colorMatcher.setConfidenceThreshold(0.9);
        
        // Looking for opposite alliance ball color to extake
        if (teamColor == Alliance.Blue) {
            colorMatcher.addColorMatch(RED_CARGO_COLOR);
        }
        else if (teamColor == Alliance.Red) {
            colorMatcher.addColorMatch(BLUE_CARGO_COLOR);
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

        solenoid.set(RETRACT.value);
    }

    /**
     * Returns true if the opposing team's cargo is present
     */
    public boolean hasOpposingColorCargo() {
        return colorMatcher.matchColor(colorSensor.getColor()) != null;
    }

    /**
     * Returns current color value detected if matched
     */
    public Color getMatchedSensorColor() {
        return colorMatcher.matchColor(colorSensor.getColor()).color;
    }

    //public void 

    @Override
    public void periodic() {
        intakeSolenoidState = IntakeSolenoidState.EXTEND;
        
    }

}