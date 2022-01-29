package frc.team2412.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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
        hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
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

    /**
     * Resets the turret motor's integrated encoder to 0.
     */
    public void resetTurretEncoder() {
        turretMotor.setSelectedSensorPosition(0);
    }

    /**
     * Sets the turret's angle to the given angle.
     * TODO: Prevent motor from ripping out wires
     * 
     * @param angle
     *            the angle (in degrees) to set the turret to (negative for
     *            counterclockwise)
     */
    public void setTurretAngle(double angle) {
        turretMotor.set(ControlMode.Position, DEGREES_TO_ENCODER_TICKS * angle);
    }
}
