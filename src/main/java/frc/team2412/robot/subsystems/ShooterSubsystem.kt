package frc.team2412.robot.subsystems

import com.ctre.phoenix.motorcontrol.can.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase

public class ShooterSubsystem(
        flywheelMotor1: TalonFX,
        flywheelMotor2: TalonFX,
        turretMotor: TalonFX,
        hoodMotor: TalonFX
) : SubsystemBase() {}
