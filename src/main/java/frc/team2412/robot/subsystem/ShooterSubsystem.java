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
import frc.team2412.robot.Hardware;
import frc.team2412.robot.util.InterpolatingTreeMap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterSubsystem extends SubsystemBase implements Loggable {
    public static class ShooterConstants {
        // Placeholder PID constants
        // TODO non-scuffed constants
        public static final double FLYWHEEL_DEFAULT_P = 0.3;
        public static final double FLYWHEEL_DEFAULT_I = 0;
        public static final double FLYWHEEL_DEFAULT_D = 0;
        public static final double FLYWHEEL_DEFAULT_F = 0.057;
        // Placeholder PID constants
        public static final double HOOD_DEFAULT_P = 0.06;
        public static final double HOOD_DEFAULT_I = 0;
        public static final double HOOD_DEFAULT_D = 0;
        public static final double HOOD_DEFAULT_F = 0.005;
        // Placeholder PID constants
        public static final double TURRET_DEFAULT_P = 0.1;
        public static final double TURRET_DEFAULT_I = 0;
        public static final double TURRET_DEFAULT_D = 0;

        // Placeholder gearing constant of 1
        public static final double FLYWHEEL_REVS_TO_ENCODER_TICKS = 2048;
        public static final double FLYWHEEL_DEGREES_TO_ENCODER_TICKS = FLYWHEEL_REVS_TO_ENCODER_TICKS / 360;
        public static final double FLYWHEEL_RPM_TO_VELOCITY = FLYWHEEL_REVS_TO_ENCODER_TICKS / (60 * 10);
        public static final double FLYWHEEL_DEFAULT_RPM = 2000;
        public static final double FLYWHEEL_DEFAULT_VELOCITY = 2000 * FLYWHEEL_RPM_TO_VELOCITY;
        public static final int FLYWHEEL_SLOT_ID = 0;

        // Placeholder gearing constant
        public static final double HOOD_REVS_TO_DEGREES = 45 / 9.78;
        public static final double MAX_HOOD_ANGLE = 60.0;
        public static final double MIN_HOOD_ANGLE = 5;
        public static final double HOOD_ANGLE_TOLERANCE = 1;

        // Estimated gearing constant of 41
        public static final double TURRET_DEGREES_TO_ENCODER_TICKS = 41 * 2048 / 360; // 233
        public static final double MIN_TURRET_ANGLE = -90;// -200; Can barely reach -139 degrees physically 115
                                                            // tested
        public static final double MAX_TURRET_ANGLE = 90;// 115; Can barely reach 210 degrees physically 245 tested
        public static final double STARTING_TURRET_ANGLE = 0;
        public static final double TURRET_ANGLE_TOLERANCE = 1;
        public static final int TURRET_SLOT_ID = 0;

        // Current limits
        public static final SupplyCurrentLimitConfiguration flywheelCurrentLimit = new SupplyCurrentLimitConfiguration(
                true, 40, 40, 500);
        public static final SupplyCurrentLimitConfiguration turretCurrentLimit = new SupplyCurrentLimitConfiguration(
                true, 10, 10, 500);
        public static final InterpolatingTreeMap dataPoints = InterpolatingTreeMap
                .fromCSV(new File(Filesystem.getDeployDirectory(), "shooterData.csv").getPath());
    }

    /* INSTANCE VARIABLES */

    // @Log.MotorController(name = "Flywheel motor 1", columnIndex = 3, rowIndex = 0)
    private final WPI_TalonFX flywheelMotor1;

    // @Log.MotorController(name = "Flywheel motor 2", columnIndex = 3, rowIndex = 1)
    private final WPI_TalonFX flywheelMotor2;

    // @Log.MotorController(name = "Turret motor", columnIndex = 3, rowIndex = 2)
    private final WPI_TalonFX turretMotor;

    private final CANSparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final SparkMaxPIDController hoodPID;

    @Config.ToggleSwitch(name = "Working command", columnIndex = 3, rowIndex = 2, width = 1, height = 1, defaultValue = true)
    public void setWorkingCommand(boolean working) {
        workingCommand = working;
    }

    public boolean workingCommand = true;

    /* SHUFFLEBOARD INSTANCE VARIABLES */
    private double flywheelTestRPM;
    @Log(name = "Target RPM", columnIndex = 8, rowIndex = 0)
    private double targetRPM;
    @Log(name = "Target velocity", columnIndex = 8, rowIndex = 1)
    private double targetVelocity;
    private double hoodTestAngle;
    private double turretAngleBias;
    private double turretTestAngle;
    private double distanceBias;

    /**
     * Constructor for shooter subsystem.
     */
    private ShooterSubsystem() {
        var hardware = Hardware.instance;

        this.flywheelMotor1 = hardware.flywheelMotor1;
        this.flywheelMotor2 = hardware.flywheelMotor2;
        this.turretMotor = hardware.turretMotor;
        this.hoodMotor = hardware.hoodMotor;
        this.hoodEncoder = hoodMotor.getEncoder();
        this.hoodPID = hoodMotor.getPIDController();
        configMotors();
    }

    /* FUNCTIONS */

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
        // turretMotor.configClosedloopRamp(10, 0);
        turretMotor.configClosedLoopPeakOutput(TURRET_SLOT_ID, 50);
        turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, TURRET_SLOT_ID, 0);
        setTurretPID(TURRET_DEFAULT_P, TURRET_DEFAULT_I, TURRET_DEFAULT_D);

        hoodMotor.restoreFactoryDefaults();
        hoodMotor.setInverted(true);
        hoodMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        hoodMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        hoodMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
                (float) (MAX_HOOD_ANGLE / HOOD_REVS_TO_DEGREES));
        hoodMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0); // Current hood setup
                                                                            // plan starts hood at
                                                                            // 0,
                                                                            // below MIN_HOOD_ANGLE
        hoodMotor.setSmartCurrentLimit(20);
        hoodMotor.setClosedLoopRampRate(1);
        hoodMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        setHoodPID(HOOD_DEFAULT_P, HOOD_DEFAULT_I, HOOD_DEFAULT_D, HOOD_DEFAULT_F);
    }

    @Override
    public void periodic() {
    }

    // PID
    @Config(name = "Flywheel PID", columnIndex = 0, rowIndex = 0, width = 1, height = 3)
    private void setFlywheelPID(@Config(name = "flywheelP", defaultValueNumeric = FLYWHEEL_DEFAULT_P) double p,
            @Config(name = "flywheelI", defaultValueNumeric = FLYWHEEL_DEFAULT_I) double i,
            @Config(name = "flywheelD", defaultValueNumeric = FLYWHEEL_DEFAULT_D) double d,
            @Config(name = "flywheelF", defaultValueNumeric = FLYWHEEL_DEFAULT_F) double f) {
        flywheelMotor1.config_kP(FLYWHEEL_SLOT_ID, p);
        flywheelMotor1.config_kI(FLYWHEEL_SLOT_ID, i);
        flywheelMotor1.config_kD(FLYWHEEL_SLOT_ID, d);
        flywheelMotor1.config_kF(FLYWHEEL_SLOT_ID, f);
    }

    @Config(name = "Hood PID", columnIndex = 1, rowIndex = 0, width = 1, height = 3)
    private void setHoodPID(@Config(name = "hoodP", defaultValueNumeric = HOOD_DEFAULT_P) double p,
            @Config(name = "hoodI", defaultValueNumeric = HOOD_DEFAULT_I) double i,
            @Config(name = "hoodD", defaultValueNumeric = HOOD_DEFAULT_D) double d,
            @Config(name = "hoodF", defaultValueNumeric = HOOD_DEFAULT_F) double f) {
        hoodPID.setP(p);
        hoodPID.setI(i);
        hoodPID.setD(d);
        hoodPID.setFF(f);
    }

    @Config(name = "Turret PID", columnIndex = 2, rowIndex = 0, width = 1, height = 2)
    private void setTurretPID(@Config(name = "turretP", defaultValueNumeric = TURRET_DEFAULT_P) double p,
            @Config(name = "turretI", defaultValueNumeric = TURRET_DEFAULT_I) double i,
            @Config(name = "turretD", defaultValueNumeric = TURRET_DEFAULT_D) double turretD) {
        turretMotor.config_kP(TURRET_SLOT_ID, p);
        turretMotor.config_kI(TURRET_SLOT_ID, i);
        turretMotor.config_kD(TURRET_SLOT_ID, turretD);
    }

    @Config(name = "Distance bias", columnIndex = 6, rowIndex = 2)
    private void setDistanceBias(double newBias) {
        distanceBias = newBias;
    }

    public double getDistanceBias() {
        return distanceBias;
    }

    // Flywheel
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
        flywheelMotor1.stopMotor();
    }

    @Config.NumberSlider(name = "Flywheel test RPM", columnIndex = 5, rowIndex = 0, min = 0, max = 6000)
    private void setFlywheelTestRPM(double newRPM) {
        flywheelTestRPM = newRPM;
    }

    public double getFlywheelTestRPM() {
        return flywheelTestRPM;
    }

    /**
     * Sets the RPM of both flywheel motors.
     *
     * @param RPM
     *            The target RPM for the flywheel motors.
     */
    @Config.NumberSlider(name = "Set flywheel", columnIndex = 3, rowIndex = 0, min = 0, max = 6000)
    public void setFlywheelRPM(double RPM) {
        targetRPM = RPM;
        setFlywheelVelocity(RPM * FLYWHEEL_RPM_TO_VELOCITY);
    }

    /**
     * Returns the RPM of the flywheel motors according to the encoder.
     *
     * @return The current RPM of the flywheel motors.
     */
    @Log(name = "Flywheel RPM", columnIndex = 7, rowIndex = 0)
    public double getFlywheelRPM() {
        return flywheelMotor1.getSelectedSensorVelocity() / FLYWHEEL_RPM_TO_VELOCITY;
    }

    /**
     * Returns the closed loop error of the flywheel motors.
     *
     * @return The closed loop error in RPM.
     */
    @Log(name = "RPM error", columnIndex = 9, rowIndex = 0)
    public double getFlywheelRPMError() {
        return flywheelMotor1.getClosedLoopError() / FLYWHEEL_RPM_TO_VELOCITY;
    }

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
    @Log(name = "Flywheel velocity", columnIndex = 7, rowIndex = 1)
    public double getFlywheelVelocity() {
        return flywheelMotor1.getSelectedSensorVelocity();
    }

    // Hood
    /**
     * Stops the hood motor.
     */
    public void stopHoodMotor() {
        hoodMotor.stopMotor();
    }

    @Config.NumberSlider(name = "Hood test angle", columnIndex = 5, rowIndex = 1, min = 0, max = MAX_HOOD_ANGLE)
    private void setHoodTestAngle(double newAngle) {
        hoodTestAngle = newAngle;
    }

    public double getHoodTestAngle() {
        return hoodTestAngle;
    }

    /**
     * Sets the target angle for the hood motor
     *
     * @param degrees
     *            Target angle for the hood motor in degrees.
     */
    @Config.NumberSlider(name = "Set hood", columnIndex = 3, rowIndex = 1, min = MIN_HOOD_ANGLE, max = MAX_HOOD_ANGLE)
    public void setHoodAngle(double degrees) {
        degrees = Math.min(Math.max(degrees, MIN_HOOD_ANGLE), MAX_HOOD_ANGLE);
        hoodPID.setReference(degrees / HOOD_REVS_TO_DEGREES, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Returns the hood's current angle (in degrees).
     *
     * @return The current angle of the hood.
     */
    @Log(name = "Hood angle", columnIndex = 7, rowIndex = 2)
    public double getHoodAngle() {
        return hoodEncoder.getPosition() * HOOD_REVS_TO_DEGREES;
    }

    /**
     * Returns whether the hood is at the given angle.
     *
     * @param angle
     *            The angle (in degrees) to compare the hood's angle to.
     * @return true if difference between hood angle and given angle is less than HOOD_ANGLE_TOLERANCE, false otherwise.
     */
    public boolean isHoodAtAngle(double angle) {
        return Math.abs(getHoodAngle() - angle) < HOOD_ANGLE_TOLERANCE;
    }

    @Log(name = "Hood speed", columnIndex = 8, rowIndex = 2)
    private double getHoodSpeed() {
        return hoodMotor.get();
    }

    /**
     * Resets the hood motor's integrated encoder to 0.
     */
    @Config(name = "Reset hood", columnIndex = 9, rowIndex = 1)
    public void resetHoodEncoder(boolean reset) {
        if (reset) {
            hoodEncoder.setPosition(0);
        }
    }

    // Turret
    /**
     * Sets the turret's target angle to the given angle.
     *
     * If angle is too far in one direction but can be reached by rotating in the other direction, the turret will turn
     * in that direction.
     *
     * @param angle
     *            The angle (in degrees) to set the turret to (negative for counterclockwise).
     */
    boolean loopToMin = false;
    boolean loopToMax = false;
    //
    // @Log.BooleanBox(name="yeet turret", columnIndex = 9, rowIndex = 4)
    // public void yeetTurret(boolean reset) {
    // if(reset){
    // turretWorking = false;
    // }
    // }
    // @Log.BooleanBox(name="add turret", columnIndex = 8, rowIndex = 4)
    // public void addTurret(boolean reset) {
    // if(reset){
    // turretWorking = true;
    // }
    // }
    public boolean turretWorking = true;

    public void setTurretAngle(double angle) {
        if (isTurretAtAngle(angle) || !turretWorking) {
            return;
        }

        if (angle > MIN_TURRET_ANGLE && angle < MAX_TURRET_ANGLE)
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

    @Config(name = "Turret angle bias", columnIndex = 5, rowIndex = 2)
    private void setTurretAngleBias(double newBias) {
        turretAngleBias = newBias;
    }

    public double getTurretAngleBias() {
        return turretAngleBias;
    }

    @Config(name = "Turret test angle")
    private void setTurretTestAngle(double newAngle) {
        turretTestAngle = newAngle;
    }

    public double getTurretTestAngle() {
        return turretTestAngle;
    }

    /**
     * Gets angle of the turret motor (horizontal swivel).
     *
     * @return Angle, in degrees.
     */
    @Log(name = "Turret angle", columnIndex = 2, rowIndex = 2)
    public double getTurretAngle() {
        return turretMotor.getSelectedSensorPosition() / TURRET_DEGREES_TO_ENCODER_TICKS;
    }

    /**
     * Returns whether the turret is at the given angle.
     *
     * @param angle
     *            The angle (in degrees) to compare the turret's angle to.
     * @return True if difference between turret angle and given angle is less than HOOD_ANGLE_TOLERANCE, False
     *         otherwise.
     */
    public boolean isTurretAtAngle(double angle) {
        return Math.abs(getTurretAngle() - angle) < TURRET_ANGLE_TOLERANCE;
    }

    /**
     * Resets the turret motor's integrated encoder to STARTING_TURRET_ANGLE.
     */
    @Config(name = "Reset turret", columnIndex = 9, rowIndex = 2)
    public void resetTurretEncoder(boolean reset) {
        if (reset) {
            turretMotor.setSelectedSensorPosition(STARTING_TURRET_ANGLE);
        }
    }

    // Singleton
    public static final ShooterSubsystem instance = new ShooterSubsystem();
}
