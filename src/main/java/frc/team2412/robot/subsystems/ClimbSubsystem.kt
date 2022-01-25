package frc.team2412.robot.subsystems

import com.ctre.phoenix.motorcontrol.TalonFXControlMode
import com.ctre.phoenix.motorcontrol.can.TalonFX
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase

public class ClimbSubsystem(
        climbFixed1: TalonFX,
        climbFixed2: TalonFX,
        climbAngle: DoubleSolenoid
) : SubsystemBase() {
    public companion object ClimbConstants {
        public const val MAX_SPEED = 1.0
        public const val STOP_SPEED = 0.0
    }

    enum class ClimbSubsystemState {
        ENABLED,
        DISABLED
    }

    enum class HookArmState {
        EXTENDED,
        UPRIGHT
    }

    private val climbMotor1 = climbFixed1
    private val climbMotor2 = climbFixed2
    private val solenoid = climbAngle

    private var state = ClimbSubsystemState.DISABLED

    public fun setEnabled() {
        state = ClimbSubsystemState.ENABLED
    }

    public fun setDisabled() {
        state = ClimbSubsystemState.DISABLED
    }

    /**
     * Extend the soldenoid to angle the climb hook
     * @param extended True to extend the arm, false to collapse
     */
    public fun angleClimbHook(extended: Boolean) {
        solenoid.set(if (extended) DoubleSolenoid.Value.kForward else DoubleSolenoid.Value.kReverse)
    }

    public fun extendFixedArm() {
        climbMotor1.set(TalonFXControlMode.PercentOutput, ClimbConstants.MAX_SPEED)
    }

    public fun retractFixedArm() {
        climbMotor1.set(TalonFXControlMode.PercentOutput, -ClimbConstants.MAX_SPEED)
    }

    public fun stopFixedArm() {
        climbMotor1.set(TalonFXControlMode.PercentOutput, ClimbConstants.STOP_SPEED)
    }

    public fun extendAngledArm() {
        climbMotor2.set(TalonFXControlMode.PercentOutput, ClimbConstants.MAX_SPEED)
    }

    public fun retractAngledArm() {
        climbMotor2.set(TalonFXControlMode.PercentOutput, -ClimbConstants.MAX_SPEED)
    }

    public fun stopAngledArm() {
        climbMotor2.set(TalonFXControlMode.PercentOutput, ClimbConstants.STOP_SPEED)
    }
}
