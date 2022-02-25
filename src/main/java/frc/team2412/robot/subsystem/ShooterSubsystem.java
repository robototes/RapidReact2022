package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.ShooterSubsystem.ShooterConstants.*;

import java.io.File;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.util.InterpolatingTreeMap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterSubsystem extends SubsystemBase implements Loggable {
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
            CANSparkMax hoodMotor) {
        this.flywheelMotor1 = flywheelMotor1;
        this.flywheelMotor2 = flywheelMotor2;
        this.turretMotor = turretMotor;
        this.hoodMotor = hoodMotor;
        this.hoodEncoder = hoodMotor.getEncoder();
        this.hoodPID = hoodMotor.getPIDController();
        configMotors();
    }

    public static class ShooterConstants {
        public static final double STOP_MOTOR = 0;
        // Placeholder gearing constant of 1
        public static final double FLYWHEEL_REVS_TO_ENCODER_TICKS = 1 * 2048;
        public static final double FLYWHEEL_DEGREES_TO_ENCODER_TICKS = FLYWHEEL_REVS_TO_ENCODER_TICKS / 360;
        public static final double FLYWHEEL_RPM_TO_VELOCITY = FLYWHEEL_REVS_TO_ENCODER_TICKS / (60 * 10);
        public static final double FLYWHEEL_DEFAULT_RPM = 2000;
        public static final double FLYWHEEL_DEFAULT_VELOCITY = 2000 * FLYWHEEL_RPM_TO_VELOCITY;
        public static final int FLYWHEEL_SLOT_ID = 0;
        // Placeholder PID constants
        // TODO non-scuffed constants
        public static final double FLYWHEEL_DEFAULT_P = 1.3;
        public static final double FLYWHEEL_DEFAULT_I = 0;
        public static final double FLYWHEEL_DEFAULT_D = 0;
        public static final double FLYWHEEL_DEFAULT_F = 0;
        // Placeholder gearing constant
        public static final double HOOD_REVS_TO_DEGREES = -45 / 9.78;
        public static final double MAX_HOOD_ANGLE = 40.0;
        public static final double MIN_HOOD_ANGLE = 5;
        public static final double HOOD_ANGLE_TOLERANCE = 1;
        // Placeholder PID constants
        public static final double HOOD_DEFAULT_P = 0.06;
        public static final double HOOD_DEFAULT_I = 0;
        public static final double HOOD_DEFAULT_D = 0;
        public static final double HOOD_DEFAULT_F = 0.005;
        // Placeholder gearing constant of 1
        public static final double TURRET_DEGREES_TO_ENCODER_TICKS = 1 * 2048 / 360;
        public static final double MIN_TURRET_ANGLE = -180;
        public static final double MAX_TURRET_ANGLE = 180;
        public static final double TURRET_ANGLE_TOLERANCE = 1;
        public static final int TURRET_SLOT_ID = 0;
        // Placeholder PID constants
        public static final double TURRET_DEFAULT_P = 0.01;
        public static final double TURRET_DEFAULT_I = 0;
        public static final double TURRET_DEFAULT_D = 0;
        public static final SupplyCurrentLimitConfiguration flywheelCurrentLimit = new SupplyCurrentLimitConfiguration(
                true, 40, 40, 500);
        public static final SupplyCurrentLimitConfiguration turretCurrentLimit = new SupplyCurrentLimitConfiguration(
                true, 10, 10, 500);
        public static final InterpolatingTreeMap dataPoints = InterpolatingTreeMap
                .fromCSV(new File(Filesystem.getDeployDirectory(), "shooterData.csv").getPath());
    }

    // Instance variables
    @Log.MotorController(name = "Flywheel motor 1")
    private final WPI_TalonFX flywheelMotor1;
    @Log.MotorController(name = "Flywheel motor 2")
    private final WPI_TalonFX flywheelMotor2;
    @Log.MotorController(name = "Turret motor")
    private final WPI_TalonFX turretMotor;
    private final CANSparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final SparkMaxPIDController hoodPID;

    @Log(name = "Hood motor speed")
    private double getHoodMotorSpeed() {
        return hoodMotor.get();
    }

    private double flywheelTestRPM;

    @Config.NumberSlider(name = "Flywheel test RPM", min = 0, max = 6000)
    private void setFlywheelTestRPM(double newRPM) {
        flywheelTestRPM = newRPM;
    }

    public double getFlywheelTestRPM() {
        return flywheelTestRPM;
    }

    private double hoodTestAngle;

    @Config.NumberSlider(name = "Hood test angle", min = 0, max = MAX_HOOD_ANGLE)
    private void setHoodTestAngle(double newAngle) {
        hoodTestAngle = newAngle;
    }

    public double getHoodTestAngle() {
        return hoodTestAngle;
    }

    private double turretAngleBias;

    @Config(name = "Turret angle bias")
    private void setTurretAngleBias(double newBias) {
        turretAngleBias = newBias;
    }

    public double getTurretAngleBias() {
        return turretAngleBias;
    }

    private double distanceBias;

    @Config(name = "Distance bias")
    private void setDistanceBias(double newBias) {
        distanceBias = newBias;
    }

    public double getDistanceBias() {
        return distanceBias;
    }

    @Config(name = "Flywheel PID")
    private void setFlywheelPID(@Config(name = "flywheelP", defaultValueNumeric = FLYWHEEL_DEFAULT_P) double p,
            @Config(name = "flywheelI", defaultValueNumeric = FLYWHEEL_DEFAULT_I) double i,
            @Config(name = "flywheelD", defaultValueNumeric = FLYWHEEL_DEFAULT_D) double d,
            @Config(name = "flywheelF", defaultValueNumeric = FLYWHEEL_DEFAULT_F) double f) {
        flywheelMotor1.config_kP(FLYWHEEL_SLOT_ID, p);
        flywheelMotor1.config_kI(FLYWHEEL_SLOT_ID, i);
        flywheelMotor1.config_kD(FLYWHEEL_SLOT_ID, d);
        flywheelMotor1.config_kF(FLYWHEEL_SLOT_ID, f);
    }

    @Config(name = "Hood PID")
    private void setHoodPID(@Config(name = "hoodP", defaultValueNumeric = HOOD_DEFAULT_P) double p,
            @Config(name = "hoodI", defaultValueNumeric = HOOD_DEFAULT_I) double i,
            @Config(name = "hoodD", defaultValueNumeric = HOOD_DEFAULT_D) double d,
            @Config(name = "hoodF", defaultValueNumeric = HOOD_DEFAULT_P) double f) {
        hoodPID.setP(p);
        hoodPID.setI(i);
        hoodPID.setD(d);
        hoodPID.setFF(f);
    }

    @Config(name = "Turret PID")
    private void setTurretPID(@Config(name = "turretP", defaultValueNumeric = TURRET_DEFAULT_P) double p,
            @Config(name = "turretI", defaultValueNumeric = TURRET_DEFAULT_I) double i,
            @Config(name = "turretD", defaultValueNumeric = TURRET_DEFAULT_D) double turretD) {
        turretMotor.config_kP(TURRET_SLOT_ID, p);
        turretMotor.config_kI(TURRET_SLOT_ID, i);
        turretMotor.config_kD(TURRET_SLOT_ID, turretD);
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
        setFlywheelPID(FLYWHEEL_DEFAULT_P, FLYWHEEL_DEFAULT_I, FLYWHEEL_DEFAULT_D, FLYWHEEL_DEFAULT_F);

        turretMotor.configFactoryDefault();
        turretMotor.configForwardSoftLimitThreshold(MAX_TURRET_ANGLE * TURRET_DEGREES_TO_ENCODER_TICKS);
        turretMotor.configReverseSoftLimitThreshold(MIN_TURRET_ANGLE * TURRET_DEGREES_TO_ENCODER_TICKS);
        turretMotor.configForwardSoftLimitEnable(true);
        turretMotor.configReverseSoftLimitEnable(true);
        turretMotor.configSupplyCurrentLimit(turretCurrentLimit);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, TURRET_SLOT_ID, 0);
        setTurretPID(TURRET_DEFAULT_P, TURRET_DEFAULT_I, TURRET_DEFAULT_D);

        hoodMotor.restoreFactoryDefaults();
        hoodMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        // hoodMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        hoodMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) MAX_HOOD_ANGLE / 360);
        // hoodMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0); // Current hood setup plan
        // starts hood at 0,
        // // below MIN_HOOD_ANGLE
        hoodMotor.setSmartCurrentLimit(40);
        hoodMotor.setClosedLoopRampRate(1);
        hoodMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        setHoodPID(HOOD_DEFAULT_P, HOOD_DEFAULT_I, HOOD_DEFAULT_D, HOOD_DEFAULT_F);
    }

    @Override
    public void periodic() {
        turretMotor.set(0);
    }

    @Log(name = "Flywheel target RPM")
    private double setRPM;

    /**
     * Sets the RPM of both flywheel motors.
     *
     * @param RPM
     *            The target RPM for the flywheel motors.
     */
    public void setFlywheelRPM(double RPM) {
        setRPM = RPM;
        setFlywheelVelocity(RPM * FLYWHEEL_RPM_TO_VELOCITY);
    }

    /**
     * Returns the RPM of the flywheel motors according to the encoder.
     *
     * @return The current RPM of the flywheel motors.
     */
    @Log(name = "Flywheel RPM")
    public double getFlywheelRPM() {
        return flywheelMotor1.getSelectedSensorVelocity() / FLYWHEEL_RPM_TO_VELOCITY;
    }

    @Log(name = "Flywheel target velocity")
    private double setVelocity;

    /**
     * Sets the velocity of both flywheel motors
     *
     * @param velocity
     *            The target velocity of the flywheel motors (in ticks per 100ms).
     */
    public void setFlywheelVelocity(double velocity) {
        flywheelMotor1.set(ControlMode.Velocity, velocity);
    }

    /**
     * Returns the velocity of the flywheel motors according to the encoder.
     *
     * @return The velocity of the flywheel motors.
     */
    @Log(name = "Flywheel velocity")
    public double getFlywheelVelocity() {
        return flywheelMotor1.getSelectedSensorVelocity();
    }

    /**
     * Returns the closed loop error of the flywheel motors.
     *
     * @return The closed loop error in RPM.
     */
    @Log(name = "Flywheel RPM error")
    public double getFlywheelRPMError() {
        return flywheelMotor1.getClosedLoopError() / FLYWHEEL_RPM_TO_VELOCITY;
    }

    /**
     * Starts both flywheel motors at the default velocity.
     */
    public void startFlywheel() {
        setFlywheelRPM(FLYWHEEL_DEFAULT_RPM);
    }

    /**
     * Stops both flywheel motors.
     */
    public void stopFlywheel() {
        setFlywheelRPM(STOP_MOTOR);
    }

    /**
     * Resets the hood motor's integrated encoder to 0.
     */
    @Config(name = "Reset hood encoder")
    public void resetHoodEncoder(boolean reset) {
        if (reset) {
            hoodEncoder.setPosition(0);
        }
    }

    /**
     * Returns the hood's current angle (in degrees).
     *
     * @return The current angle of the hood.
     */
    @Log(name = "Hood angle")
    public double getHoodAngle() {
        return hoodEncoder.getPosition() * HOOD_REVS_TO_DEGREES;
    }

    /**
     * Returns whether the hood is at the given angle.
     *
     * @param angle
     *            The angle (in degrees) to compare the hood's angle to.
     * @return true if difference between hood angle and given angle is less than HOOD_ANGLE_TOLERANCE,
     *         false otherwise.
     */
    public boolean isHoodAtAngle(double angle) {
        return Math.abs(getHoodAngle() - angle) < HOOD_ANGLE_TOLERANCE;
    }

    /**
     * Sets the target angle for the hood motor
     *
     * @param degrees
     *            Target angle for the hood motor in degrees.
     */
    public void setHoodAngle(double degrees) {
        degrees = Math.min(Math.max(degrees, MIN_HOOD_ANGLE), MAX_HOOD_ANGLE);
        hoodPID.setReference(degrees / HOOD_REVS_TO_DEGREES, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Stops the hood motor.
     */
    public void stopHoodMotor() {
        hoodMotor.set(STOP_MOTOR);
    }

    /**
     * Resets the turret motor's integrated encoder to 0.
     */
    @Config(name = "Reset turret encoder")
    public void resetTurretEncoder(boolean reset) {
        if (reset) {
            turretMotor.setSelectedSensorPosition(0);
        }
    }

    /**
     * Gets angle of the turret motor (horizontal swivel).
     *
     * @return Angle, in degrees.
     */
    public double getTurretAngle() {
        return turretMotor.getSelectedSensorPosition() / TURRET_DEGREES_TO_ENCODER_TICKS;
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
     * If angle is too far in one direction but can be reached by rotating in the other direction, the
     * turret will turn in that direction.
     *
     * @param angle
     *            The angle (in degrees) to set the turret to (negative for counterclockwise).
     */
    public void setTurretAngle(double angle) {
        if (angle < MIN_TURRET_ANGLE) {
            System.out.println("LOG: Desired turret angle is below min angle!");
            if (angle + 360 < MAX_TURRET_ANGLE) {
                System.out.println("LOG: Targeting desired turret angle in other direction...");
                angle += 360;
            } else {
                System.out.println("LOG: Couldn't wrap around turret angle!");
            }
        } else if (angle > MAX_TURRET_ANGLE) {
            System.out.println("LOG: Desired turret angle is above max angle!");
            if (angle - 360 > MIN_TURRET_ANGLE) {
                System.out.println("LOG: Targeting desired turret angle in other direction...");
                angle -= 360;
            } else {
                System.out.println("LOG: Couldn't wrap around turret angle!");
            }
        }
        turretMotor.set(ControlMode.Position, TURRET_DEGREES_TO_ENCODER_TICKS * angle);
    }

    /**
     * Sets the turret angle relative to the current position of the motor (not the last target angle).
     *
     * @param deltaAngle
     *            Amount to change the turret angle by (in degrees).
     */
    public void updateTurretAngle(double deltaAngle) {
        setTurretAngle(getTurretAngle() + deltaAngle);
    }

}
