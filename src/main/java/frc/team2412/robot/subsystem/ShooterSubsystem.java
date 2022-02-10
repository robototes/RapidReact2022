package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.ShooterSubsystem.ShooterConstants.*;
import frc.team2412.robot.util.InterpolatingTreeMap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase implements Loggable {
    public static class ShooterConstants {
        public static final double DEGREES_TO_ENCODER_TICKS = 2048 / 360;
        public static final double DEFAULT_FLYWHEEL_VELOCITY = 10;
        public static final double STOP_MOTOR = 0;
        public static final int FLYWHEEL_SLOT_ID = 0;
        // Placeholder PID constants
        public static final double FLYWHEEL_P = 0.01;
        public static final double FLYWHEEL_I = 0;
        public static final double FLYWHEEL_D = 0;
        public static final double MIN_TURRET_ANGLE = -180;
        public static final double MAX_TURRET_ANGLE = 180;
        public static final double TURRET_ANGLE_TOLERANCE = 1;
        public static final int TURRET_SLOT_ID = 0;
        // Placeholder PID constants
        public static final double TURRET_P = 0.01;
        public static final double TURRET_I = 0;
        public static final double TURRET_D = 0;
        public static final double TURRET_ANGLE_BIAS = 0; // Move to subsystem instance field to configure
        public static final double MAX_HOOD_ANGLE = 40.0;
        public static final double MIN_HOOD_ANGLE = 5;
        public static final double HOOD_ANGLE_TOLERANCE = 1;
        public static final SupplyCurrentLimitConfiguration flywheelCurrentLimit = new SupplyCurrentLimitConfiguration(
                true,
                40, 40, 500);
        public static final SupplyCurrentLimitConfiguration turretCurrentLimit = new SupplyCurrentLimitConfiguration(
                true,
                10, 10, 500);
        public static final SupplyCurrentLimitConfiguration hoodCurrentLimit = turretCurrentLimit;
        public static final double DISTANCE_BIAS = 0; // Move to subsystem instance field to configure
        public static final InterpolatingTreeMap dataPoints = new InterpolatingTreeMap();
    }

    // Instance variables
    @Log.MotorController
    private final WPI_TalonFX flywheelMotor1;
    @Log.MotorController
    private final WPI_TalonFX flywheelMotor2;
    @Log.MotorController
    private final WPI_TalonFX turretMotor;
    @Log.MotorController
    private final WPI_TalonFX hoodMotor;

    @Config
    private double flywheelTestVelocity; // TODO: Make actually show up in shuffleboard

    public double getFlywheelTestVelocity() {
        return flywheelTestVelocity;
    }

    @Config
    private double hoodTestAngle;

    public double getHoodTestAngle() {
        return hoodTestAngle;
    }

    @Config
    private void setFlywheelPID(@Config(name = "flywheelP", defaultValueNumeric = FLYWHEEL_P) double p,
            @Config(name = "flywheelI", defaultValueNumeric = FLYWHEEL_I) double i,
            @Config(name = "flywheelD", defaultValueNumeric = FLYWHEEL_D) double d) {
        flywheelMotor1.config_kP(FLYWHEEL_SLOT_ID, p);
        flywheelMotor1.config_kI(FLYWHEEL_SLOT_ID, i);
        flywheelMotor1.config_kD(FLYWHEEL_SLOT_ID, d);
    }

    @Config
    private void setTurretPID(@Config(name = "turretP", defaultValueNumeric = TURRET_P) double p,
            @Config(name = "turretI", defaultValueNumeric = TURRET_I) double i,
            @Config(name = "turretD", defaultValueNumeric = TURRET_D) double turretD) {
        turretMotor.config_kP(TURRET_SLOT_ID, p);
        turretMotor.config_kI(TURRET_SLOT_ID, i);
        turretMotor.config_kD(TURRET_SLOT_ID, turretD);
    }

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
        this.flywheelMotor1 = flywheelMotor1;
        this.flywheelMotor2 = flywheelMotor2;
        this.turretMotor = turretMotor;
        this.hoodMotor = hoodMotor;
        configMotors();
    }

    /**
     * Configures the instance motors
     */
    private void configMotors() {
        flywheelMotor1.configFactoryDefault();
        flywheelMotor1.configSupplyCurrentLimit(flywheelCurrentLimit);
        flywheelMotor1.setNeutralMode(NeutralMode.Coast);
        flywheelMotor2.configFactoryDefault();
        flywheelMotor2.configSupplyCurrentLimit(flywheelCurrentLimit);
        flywheelMotor2.setNeutralMode(NeutralMode.Coast);
        flywheelMotor1.setInverted(false);
        flywheelMotor2.follow(flywheelMotor1);
        flywheelMotor2.setInverted(InvertType.OpposeMaster);
        setFlywheelPID(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D);

        turretMotor.configFactoryDefault();
        turretMotor.configForwardSoftLimitThreshold(MAX_TURRET_ANGLE * DEGREES_TO_ENCODER_TICKS);
        turretMotor.configReverseSoftLimitThreshold(MIN_TURRET_ANGLE * DEGREES_TO_ENCODER_TICKS);
        turretMotor.configForwardSoftLimitEnable(true);
        turretMotor.configReverseSoftLimitEnable(true);
        turretMotor.configSupplyCurrentLimit(turretCurrentLimit);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, TURRET_SLOT_ID, 0);
        setTurretPID(TURRET_P, TURRET_I, TURRET_D);

        hoodMotor.configFactoryDefault();
        hoodMotor.configForwardSoftLimitThreshold(MAX_HOOD_ANGLE * DEGREES_TO_ENCODER_TICKS);
        hoodMotor.configReverseSoftLimitThreshold(0); // Current hood setup plan starts hood at 0, below MIN_HOOD_ANGLE
        hoodMotor.configForwardSoftLimitEnable(true);
        hoodMotor.configReverseSoftLimitEnable(true);
        hoodMotor.configSupplyCurrentLimit(hoodCurrentLimit);
        hoodMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets the velocity of both flywheel motors
     *
     * @param velocity
     *            the velocity of the flywheel motors
     */
    public void setFlywheelVelocity(double velocity) {
        flywheelMotor1.set(ControlMode.Velocity, velocity);
    }

    /**
     * Starts both flywheel motors
     */
    public void startFlywheel() {
        flywheelMotor1.set(ControlMode.Velocity, DEFAULT_FLYWHEEL_VELOCITY);
    }

    /**
     * Stops both flywheel motors
     */
    public void stopFlywheel() {
        flywheelMotor1.set(STOP_MOTOR);
    }

    /**
     * Resets the hood motor's integrated encoder to 0.
     */
    public void resetHoodEncoder() {
        hoodMotor.setSelectedSensorPosition(0);
    }

    /**
     * Returns the hood's current angle (in degrees).
     *
     * @return The current angle of the hood
     */
    public double getHoodAngle() {
        return hoodMotor.getSelectedSensorPosition() / DEGREES_TO_ENCODER_TICKS;
    }

    /**
     * Returns whether the hood is at the given angle.
     *
     * @param angle
     *            The angle (in degrees) to compare the hood's angle to.
     * @return True if difference between hood angle and given angle is less than HOOD_ANGLE_TOLERANCE,
     *         False otherwise.
     */
    public boolean isHoodAtAngle(double angle) {
        return Math.abs(getHoodAngle() - angle) < HOOD_ANGLE_TOLERANCE;
    }

    /**
     * Sets the target angle for the hood motor
     *
     * @param degrees
     *            Target angle for the hood motor in degrees
     */
    public void setHoodAngle(double degrees) {
        degrees = Math.min(Math.max(degrees, MIN_HOOD_ANGLE), MAX_HOOD_ANGLE);
        hoodMotor.set(ControlMode.Position, DEGREES_TO_ENCODER_TICKS * degrees);
    }

    /**
     * Stops the hood motor
     */
    public void stopHoodMotor() {
        hoodMotor.set(STOP_MOTOR);
    }

    /**
     * Resets the turret motor's integrated encoder to 0.
     */
    public void resetTurretEncoder() {
        turretMotor.setSelectedSensorPosition(0);
    }

    /**
     * Gets angle of the turret motor (horizontal swivel)
     *
     * @return Angle, in degrees
     */
    public double getTurretAngle() {
        return turretMotor.getSelectedSensorPosition() / DEGREES_TO_ENCODER_TICKS;
    }

    /**
     * Returns whether the turret is at the given angle.
     *
     * @param angle
     *            The angle (in degrees) to compare the turret's angle to.
     * @return True if difference between turret angle and given angle is less than
     *         HOOD_ANGLE_TOLERANCE, False otherwise.
     */
    public boolean isTurretAtAngle(double angle) {
        return Math.abs(getTurretAngle() - angle) < TURRET_ANGLE_TOLERANCE;
    }

    /**
     * Sets the turret's target angle to the given angle.
     *
     * If angle is too far in one direction but can be reached by rotating in the
     * other direction, the turret will turn in that direction.
     *
     * @param angle
     *            the angle (in degrees) to set the turret to (negative for
     *            counterclockwise)
     */
    public void setTurretAngle(double angle) {
        if (angle < MIN_TURRET_ANGLE) {
            System.out.println("LOG: Desired turret angle is below min angle");
            if (angle + 360 < MAX_TURRET_ANGLE) {
                System.out.println("LOG: Targeting desired turret angle in other direction");
                angle += 360;
            }
        } else if (angle > MAX_TURRET_ANGLE) {
            System.out.println("LOG: Desired turret angle is above max angle");
            if (angle - 360 > MIN_TURRET_ANGLE) {
                System.out.println("LOG: Targeting desired turret angle in other direction");
                angle -= 360;
            }
        }
        turretMotor.set(ControlMode.Position, DEGREES_TO_ENCODER_TICKS * angle);
    }

    /**
     * Sets the turret angle realative to the current angle - not the last target
     * angle, but the current position of the motor
     *
     * @param deltaAngle
     *            Amount to change the turret angle by in degrees
     */
    public void updateTurretAngle(double deltaAngle) {
        setTurretAngle(getTurretAngle() + deltaAngle);
    }
}
