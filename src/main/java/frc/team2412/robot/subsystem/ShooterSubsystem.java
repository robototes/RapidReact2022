package frc.team2412.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // instance variables
    private final WPI_TalonFX flywheelMotor1;
    private final WPI_TalonFX flywheelMotor2;
    private final WPI_TalonFX turretMotor;
    private final WPI_TalonFX hoodMotor;

    // Constants
    public static final double MAX_FORWARD = 0.1;
    public static final double FLYWHEEL_VELOCITY = 10;
    public static final double MAX_REVERSE = -0.1;
    public static final double STOP_MOTOR = 0;
    public static final double DEGREES_TO_ENCODER_TICKS = 2048 / 360; // 2048 ticks per 360 degrees
    public static final double MIN_TURRET_ANGLE = -180; // Total ~360 degrees of rotation, assumes 0 is center
    public static final double MAX_TURRET_ANGLE = 180;
    public static final double TURRET_MAX_SPEED = 0.1;
    public static final double TURRET_MIN_SPEED = -0.1;
    public static final double TURRET_P = 0.01; // Placeholder PID constants
    public static final double TURRET_I = 0;
    public static final double TURRET_D = 0;
    public static final double MAX_HOOD_ANGLE = 40.0;
    public static final double MIN_HOOD_ANGLE = 5;

    /**
     * Constructor for shooter subsystem.
     * 
     * @param flywheelMotor1
     *            The first motor connected to the flywheel
     * 
     * @param flywheelMotor2
     *            The second motor connected to the flywheel
     * 
     * @param turretMotor
     *            The motor that controls the horizontal rotation of the
     *            turret
     * 
     * @param hoodMotor
     *            The motor that controls the angle of the hood above the
     *            turret
     * 
     */
    public ShooterSubsystem(WPI_TalonFX flywheelMotor1, WPI_TalonFX flywheelMotor2, WPI_TalonFX turretMotor,
            WPI_TalonFX hoodMotor) {
        // Motor configs
        var flywheelLimit = new SupplyCurrentLimitConfiguration(true, 40, 40, 500);
        flywheelMotor1.configSupplyCurrentLimit(flywheelLimit);
        flywheelMotor1.setNeutralMode(NeutralMode.Coast);
        flywheelMotor2.configSupplyCurrentLimit(flywheelLimit);
        flywheelMotor2.setNeutralMode(NeutralMode.Coast);
        flywheelMotor2.setInverted(true);

        var limit = new SupplyCurrentLimitConfiguration(true, 10, 10, 500);
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretConfig.forwardSoftLimitThreshold = MAX_TURRET_ANGLE * DEGREES_TO_ENCODER_TICKS;
        turretConfig.reverseSoftLimitThreshold = MIN_TURRET_ANGLE * DEGREES_TO_ENCODER_TICKS;
        turretConfig.forwardSoftLimitEnable = true;
        turretConfig.reverseSoftLimitEnable = true;
        turretMotor.configSupplyCurrentLimit(limit);
        turretMotor.configAllSettings(turretConfig);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        turretMotor.config_kP(0, TURRET_P);
        turretMotor.config_kI(0, TURRET_I);
        turretMotor.config_kD(0, TURRET_D);
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.forwardSoftLimitThreshold = MAX_HOOD_ANGLE * DEGREES_TO_ENCODER_TICKS;
        hoodConfig.reverseSoftLimitThreshold = MIN_HOOD_ANGLE * DEGREES_TO_ENCODER_TICKS;
        hoodConfig.forwardSoftLimitEnable = true;
        hoodConfig.reverseSoftLimitEnable = true;
        hoodMotor.configSupplyCurrentLimit(limit);
        hoodMotor.configAllSettings(hoodConfig);
        hoodMotor.setNeutralMode(NeutralMode.Brake);

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

    // TODO make hardstop
    public void hoodMotorStop() {
        hoodMotor.set(STOP_MOTOR);
    }

    // TODO make sure these motors are going in the right direction
    public void startFlywheel() {
        flywheelMotor1.set(ControlMode.Velocity, FLYWHEEL_VELOCITY);
        flywheelMotor2.set(ControlMode.Velocity, FLYWHEEL_VELOCITY);
    }

    public void stopFlywheel() {
        flywheelMotor1.set(STOP_MOTOR);
        flywheelMotor2.set(STOP_MOTOR);
    }

    /**
     * Resets the turret motor's integrated encoder to 0.
     */
    // TODO use limit switches to reset the encoder
    public void resetTurretEncoder() {
        turretMotor.setSelectedSensorPosition(0);
    }

    public double getTurretAngle() {
        return turretMotor.getSelectedSensorPosition() / DEGREES_TO_ENCODER_TICKS;
    }

    /**
     * Sets the turret's angle to the given angle (Does not check angle limits).
     * 
     * @param angle
     *            the angle (in degrees) to set the turret to (negative for
     *            counterclockwise)
     */
    public void setTurretAngle(double angle) {
        turretMotor.set(ControlMode.Position, DEGREES_TO_ENCODER_TICKS * angle);
    }

    public void updateTurretAngle(double deltaAngle) {
        double currentAngle = turretMotor.getSelectedSensorPosition() / DEGREES_TO_ENCODER_TICKS;
        setTurretAngle(currentAngle + deltaAngle);
    }
}
