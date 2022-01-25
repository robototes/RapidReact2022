package frc.team2412.robot.subsystems

import com.ctre.phoenix.motorcontrol.can.TalonFX
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase

public class IntakeSubsystem(
        intakeMotor1: TalonFX,
        intakeMotor2: TalonFX,
        intakeSolenoid: DoubleSolenoid
) : SubsystemBase() {}
