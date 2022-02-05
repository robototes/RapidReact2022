package frc.team2412.robot.subsystem;

import static frc.team2412.robot.subsystem.ShooterSubsystem.ShooterConstants.*;
import frc.team2412.robot.util.InterpolatingTreeMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public static class ShooterConstants {
        public static final double DEGREES_TO_ENCODER_TICKS = 2048 / 360;
        public static final double FLYWHEEL_VELOCITY = 10;
        public static final double STOP_MOTOR = 0;
        public static final int FLYWHEEL_SLOT_ID = 0;
        // Placeholder PID constants
        public static final double FLYWHEEL_P = 0.01;
        public static final double FLYWHEEL_I = 0;
        public static final double FLYWHEEL_D = 0;
        public static final double MIN_TURRET_ANGLE = -180;
        public static final double MAX_TURRET_ANGLE = 180;
        public static final int TURRET_SLOT_ID = 0;
        // Placeholder PID constants
        public static final double TURRET_P = 0.01;
        public static final double TURRET_I = 0;
        public static final double TURRET_D = 0;
        public static final double MAX_HOOD_ANGLE = 40.0;
        public static final double MIN_HOOD_ANGLE = 5;
        public static final SupplyCurrentLimitConfiguration flywheelCurrentLimit = new SupplyCurrentLimitConfiguration(
                true,
                40, 40, 500);
        public static final SupplyCurrentLimitConfiguration turretCurrentLimit = new SupplyCurrentLimitConfiguration(
                true,
                10, 10, 500);
        public static final SupplyCurrentLimitConfiguration hoodCurrentLimit = turretCurrentLimit;
        public static final InterpolatingTreeMap dataPoints = new InterpolatingTreeMap();
        public static final ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        public static final NetworkTableEntry flywheelPEntry = tab.add("Flywheel P", FLYWHEEL_P).withPosition(0, 0)
                .withSize(1, 1).getEntry();
        public static final NetworkTableEntry flywheelIEntry = tab.add("Flywheel I", FLYWHEEL_I).withPosition(0, 1)
                .withSize(1, 1).getEntry();
        public static final NetworkTableEntry flywheelDEntry = tab.add("Flywheel D", FLYWHEEL_D).withPosition(0, 2)
                .withSize(1, 1).getEntry();
        public static final NetworkTableEntry turretPEntry = tab.add("Turret P", TURRET_P).withPosition(1, 0)
                .withSize(1, 1).getEntry();
        public static final NetworkTableEntry turretIEntry = tab.add("Turret I", TURRET_I).withPosition(1, 1)
                .withSize(1, 1).getEntry();
        public static final NetworkTableEntry turretDEntry = tab.add("Turret D", TURRET_D).withPosition(1, 2)
                .withSize(1, 1).getEntry();
        public static final NetworkTableEntry distanceBiasEntry = tab.add("Distance Bias", 0.0).withPosition(2, 0)
                .withSize(1, 1).getEntry();
    }

    /**
     * Returns the flywheel P value.
     *
     * @return The flywheel P value
     */
    public static double getFlywheelP() {
        return flywheelPEntry.getDouble(FLYWHEEL_P);
    }

    /**
     * Returns the flywheel I value.
     *
     * @return The flywheel I value
     */
    public static double getFlywheelI() {
        return flywheelIEntry.getDouble(FLYWHEEL_I);
    }

    /**
     * Returns the flywheel D value.
     *
     * @return The flywheel D value
     */
    public static double getFlywheelD() {
        return flywheelDEntry.getDouble(FLYWHEEL_D);
    }

    /**
     * Returns the turret P value.
     *
     * @return The turret P value
     */
    public static double getTurretP() {
        return turretPEntry.getDouble(TURRET_P);
    }

    /**
     * Returns the turret I value.
     *
     * @return The turret I value
     */
    public static double getTurretI() {
        return turretIEntry.getDouble(TURRET_I);
    }

    /**
     * Returns the turret D value.
     *
     * @return The turret D value
     */
    public static double getTurretD() {
        return turretDEntry.getDouble(TURRET_D);
    }

    // Instance variables
    private final WPI_TalonFX flywheelMotor1;
    private final WPI_TalonFX flywheelMotor2;
    private final WPI_TalonFX turretMotor;
    private final WPI_TalonFX hoodMotor;

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
        flywheelMotor1.config_kP(FLYWHEEL_SLOT_ID, getFlywheelP());
        flywheelMotor1.config_kI(FLYWHEEL_SLOT_ID, getFlywheelI());
        flywheelMotor1.config_kD(FLYWHEEL_SLOT_ID, getFlywheelD());
        flywheelMotor2.configFactoryDefault();
        flywheelMotor2.configSupplyCurrentLimit(flywheelCurrentLimit);
        flywheelMotor2.setNeutralMode(NeutralMode.Coast);
        flywheelMotor1.setInverted(false);
        flywheelMotor2.follow(flywheelMotor1);
        flywheelMotor2.setInverted(InvertType.OpposeMaster);

        turretMotor.configFactoryDefault();
        turretMotor.configForwardSoftLimitThreshold(MAX_TURRET_ANGLE * DEGREES_TO_ENCODER_TICKS);
        turretMotor.configReverseSoftLimitThreshold(MIN_TURRET_ANGLE * DEGREES_TO_ENCODER_TICKS);
        turretMotor.configForwardSoftLimitEnable(true);
        turretMotor.configReverseSoftLimitEnable(true);
        turretMotor.configSupplyCurrentLimit(turretCurrentLimit);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, TURRET_SLOT_ID, 0);
        turretMotor.config_kP(TURRET_SLOT_ID, getTurretP());
        turretMotor.config_kI(TURRET_SLOT_ID, getTurretI());
        turretMotor.config_kD(TURRET_SLOT_ID, getTurretD());

        hoodMotor.configFactoryDefault();
        hoodMotor.configForwardSoftLimitThreshold(MAX_HOOD_ANGLE * DEGREES_TO_ENCODER_TICKS);
        hoodMotor.configReverseSoftLimitThreshold(0); // Current hood setup plan starts hood at 0, below MIN_HOOD_ANGLE
        hoodMotor.configForwardSoftLimitEnable(true);
        hoodMotor.configReverseSoftLimitEnable(true);
        hoodMotor.configSupplyCurrentLimit(hoodCurrentLimit);
        hoodMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets the target angle for the hood motor
     *
     * @param degrees
     *            Target angle for the hood motor in degrees
     */
    public void hoodMotorSetAngle(double degrees) {
        degrees = Math.min(Math.max(degrees, MIN_HOOD_ANGLE), MAX_HOOD_ANGLE);
        hoodMotor.set(ControlMode.Position, DEGREES_TO_ENCODER_TICKS * degrees);
    }

    /**
     * Stops the hood motor
     */
    // TODO make hardstop
    public void hoodMotorStop() {
        hoodMotor.set(STOP_MOTOR);
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
        flywheelMotor1.set(ControlMode.Velocity, FLYWHEEL_VELOCITY);
    }

    /**
     * Stops both flywheel motors
     */
    public void stopFlywheel() {
        flywheelMotor1.set(STOP_MOTOR);
    }

    /**
     * Resets the turret motor's integrated encoder to 0.
     */
    // TODO use limit switches to reset the encoder
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
