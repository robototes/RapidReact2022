package frc.team2412.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final WPI_TalonFX flywheelMotor1;
    private final WPI_TalonFX flywheelMotor2;
    private final WPI_TalonFX turretMotor;
    private final WPI_TalonFX hoodMotor;

    // TODO remove the MAX_VALUE (it was funny)
    public static final double MAX_FORWARD = Double.MAX_VALUE;
    public static final double MAX_REVERSE = -1;
    public static final double STOP_MOTOR = 0;

    public ShooterSubsystem(WPI_TalonFX flywheelMotor1, WPI_TalonFX flywheelMotor2, WPI_TalonFX turretMotor,
            WPI_TalonFX hoodMotor) {
        this.flywheelMotor1 = flywheelMotor1;
        this.flywheelMotor2 = flywheelMotor2;
        this.turretMotor = turretMotor;
        this.hoodMotor = hoodMotor;
    }

    public void hoodMotorExtend() {
        hoodMotor.set(MAX_FORWARD);
    }

    public void hoodMotorRetract() {
        hoodMotor.set(MAX_REVERSE);
    }

    public void hoodMotorStop() {
        hoodMotor.set(STOP_MOTOR);
    }

    public void startFlywheel() {
        flywheelMotor1.set(MAX_FORWARD);
        flywheelMotor2.set(MAX_FORWARD);
    }

    public void stopFlywheel() {
        flywheelMotor1.set(STOP_MOTOR);
        flywheelMotor2.set(STOP_MOTOR);
    }
}
