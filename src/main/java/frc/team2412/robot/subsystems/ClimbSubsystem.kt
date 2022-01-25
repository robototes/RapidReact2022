package frc.team2412.robot.subsystems

import com.ctre.phoenix.motorcontrol.can.TalonFX
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase

public class ClimbSubsystem(
        climbFixed1: TalonFX,
        climbFixed2: TalonFX,
        climbAngled1: TalonFX,
        climbAngled2: TalonFX,
        climbAngle: DoubleSolenoid
) : SubsystemBase() {}
