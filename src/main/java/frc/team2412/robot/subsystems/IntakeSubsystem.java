package frc.team2412.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

public class IntakeSubsystem extends SubsystemBase {

    // Constants

    public static class IntakeConstants {
        public static String teamColor = "placeholder";
    }

    // Enums

    public enum IntakeMotorState{
        IN, OUT, STOPPED;
    }

    public enum IntakeSolenoidState{
        EXTENDED, RETRACTED
    }
    
    // Define Hardware

    private final TalonFX motor1;

    private final TalonFX motor2;

    private final DoubleSolenoid solenoid1;

    private final DoubleSolenoid solenoid2;

    private final ColorSensorV3 colorSensor;

    private IntakeMotorState intakeMotorState;

    private IntakeSolenoidState intakeSolenoidState;

    // CONSTRUCTOR!

    public IntakeSubsystem(TalonFX intakeMotor1, TalonFX intakeMotor2, DoubleSolenoid intakeSolenoid1, DoubleSolenoid intakeSolenoid2, ColorSensorV3 intakeColorSensor) {
        this.motor1 = intakeMotor1;
        this.motor2 = intakeMotor2;
        this.solenoid1 = intakeSolenoid1;
        this.solenoid2 = intakeSolenoid2;
        this.colorSensor = intakeColorSensor;
        intakeSolenoidState = IntakeSolenoidState.RETRACTED;
    }

    public boolean checkMotorState() {
        return true;
    }

    public void intakeIn() {
        intakeMotorState = IntakeMotorState.IN;
    }

    public void intakeOut() {
        intakeMotorState = IntakeMotorState.OUT;
    }
    public void intakeStop() {
        intakeMotorState = IntakeMotorState.STOPPED;
        
    public void intakeExtend() {
        intakeSolenoidState = IntakeSolenoidState.EXTENDED;
    }
    
    public void intakeRetract() {
        intakeSolenoidState = IntakeSolenoidState.RETRACTED;
    }

    
    }

    public boolean colorIdentify() {
        return true;
    }
    

}
