package frc.team2412.robot.subsystem

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase

public class IntakeSubsystem(
        intakeMotor1: WPI_TalonFX,
        intakeMotor2: WPI_TalonFX,
        intakeSolenoid1: Solenoid,
        intakeSolenoid2: Solenoid,
        intakeColorSensor: ColorSensorV3
) : SubsystemBase() {
    public companion object IntakeConstants {
        public const val teamColor = "placeholder"
    }

    public enum class IntakeMotorState {
        IN,
        OUT,
        STOPPED
    }

    public enum class IntakeSolenoidState {
        EXTENDED,
        RETRACTED
    }

    public val motor1 = intakeMotor1
    public val motor2 = intakeMotor2
    public val solenoid1 = intakeSolenoid1
    public val solenoid2 = intakeSolenoid2
    public val colorSensor = intakeColorSensor

    private var intakeMotorState = IntakeMotorState.IN
    private var intakeSolenoidState = IntakeSolenoidState.RETRACTED

    public fun checkMotorState(): Boolean {
        return true
    }

    public fun intakeIn() {
        intakeMotorState = IntakeMotorState.IN
    }

    public fun intakeOut() {
        intakeMotorState = IntakeMotorState.OUT
    }

    public fun intakeStop() {
        intakeMotorState = IntakeMotorState.STOPPED
    }

    public fun intakeExtend() {
        intakeSolenoidState = IntakeSolenoidState.EXTENDED
    }

    public fun intakeRetract() {
        intakeSolenoidState = IntakeSolenoidState.RETRACTED
    }

    public fun colorIdentify(): Boolean {
        return true
    }
}
