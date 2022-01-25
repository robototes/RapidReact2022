package frc.team2412.robot.subsystem

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase

public class ShooterSubsystem(
        flywheelMotor1: WPI_TalonFX,
        flywheelMotor2: WPI_TalonFX,
        turretMotor: WPI_TalonFX,
        hoodMotor: WPI_TalonFX
) : SubsystemBase() {}
