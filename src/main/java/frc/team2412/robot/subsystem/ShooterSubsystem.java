package frc.team2412.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private WPI_TalonFX flywheelMotor1;
    private WPI_TalonFX flywheelMotor2;
    private WPI_TalonFX turretMotor;
    private WPI_TalonFX hoodMotor;

    public ShooterSubsystem(WPI_TalonFX flywheelMotor1, WPI_TalonFX flywheelMotor2, WPI_TalonFX turretMotor,
            WPI_TalonFX hoodMotor) {
        this.flywheelMotor1 = flywheelMotor1;
        this.flywheelMotor2 = flywheelMotor2;
        this.turretMotor = turretMotor;
        this.hoodMotor = hoodMotor;
    }

    public void startFlywheel() {
        flywheelMotor1.set(Double.MAX_VALUE);
    }
}
