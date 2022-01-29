package frc.team2412.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

public class IntakeSubsystem extends SubsystemBase {

    // Constants

    public static class IntakeConstants {
        public static String teamColor = "placeholder";
        public static final double INTAKE_SPEED = 1;

    }

    // Enums

    public enum IntakeMotorState {
        IN, OUT, STOPPED;
    }

    public enum IntakeSolenoidState{
        EXTENDED, RETRACTED;
    }
    
    
    public enum IntakeState{
        EXTEND(DoubleSolenoid.Value.kForward),
        RETRACT(DoubleSolenoid.Value.kReverse);

        public DoubleSolenoid.Value value;

        private IntakeState(DoubleSolenoid.Value value) {
            this.value = value;
        }
    }
    


    // Define Hardware

    private final WPI_TalonFX motor1;

    private final WPI_TalonFX motor2;

    private final DoubleSolenoid solenoid1;

    //private final DoubleSolenoid solenoid2;

    //private final ColorSensorV3 colorSensor = new colorSensorV3(i2cPort);

    private IntakeMotorState intakeMotorState;

    private IntakeSolenoidState intakeSolenoidState;

    // CONSTRUCTOR!
    public IntakeSubsystem(WPI_TalonFX intakeMotor1, WPI_TalonFX intakeMotor2, DoubleSolenoid intakeSolenoid1) {
        this.motor1 = intakeMotor1;
        this.motor2 = intakeMotor2;

        this.motor2.setInverted(true);

        this.solenoid1 = intakeSolenoid1;
        //this.colorSensor = intakeColorSensor;
        intakeSolenoidState = IntakeSolenoidState.RETRACTED;
        intakeMotorState = IntakeMotorState.STOPPED;
        setName("IntakeSubsystem");
    }

    // Methods.

    public boolean checkMotorState() {
        return true;
    }

    public void intakeIn() {
        motor1.set(IntakeConstants.INTAKE_SPEED);
        motor2.set(IntakeConstants.INTAKE_SPEED);
        intakeMotorState = IntakeMotorState.IN;
    }

    public void intakeOut() {
        motor1.set(-IntakeConstants.INTAKE_SPEED);
        motor2.set(-IntakeConstants.INTAKE_SPEED);
        intakeMotorState = IntakeMotorState.OUT;
    }

    public void intakeStop() {
        motor1.set(0);
        motor2.set(0);
        intakeMotorState = IntakeMotorState.STOPPED;
    }

    public void intakeExtend() {
        intakeSolenoidState = IntakeSolenoidState.EXTENDED;

        solenoid1.set(IntakeState.EXTEND.value);
    }
    
    public void intakeRetract() {
        intakeSolenoidState = IntakeSolenoidState.RETRACTED; 

        solenoid1.set(IntakeState.RETRACT.value);
    }

    public boolean colorIdentify() {
        return true;
    }
    
}
