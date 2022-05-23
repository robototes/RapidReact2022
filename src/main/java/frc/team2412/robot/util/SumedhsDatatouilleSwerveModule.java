package frc.team2412.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SumedhsDatatouilleSwerveModule {

    private static final ModuleConfiguration moduleType = SdsModuleConfigurations.MK4_L1;
    private static final Mk4ModuleConfiguration moduleConfigs = new Mk4ModuleConfiguration();

    private static final double ENCODER_TICKS_PER_ROTATION = 2048;
    private static final double DRIVE_MOTOR_ENCODER_TICKS_TO_METERS = 1 / ENCODER_TICKS_PER_ROTATION
            * moduleType.getDriveReduction() * Math.PI * moduleType.getWheelDiameter();
    private static final double DRIVE_MOTOR_ENCODER_VELOCITY_TO_METERS_PER_SECOND = DRIVE_MOTOR_ENCODER_TICKS_TO_METERS
            * 10.0;

    private static final double TURN_MOTOR_ENCODER_TICKS_TO_DEGREE = 1 / ENCODER_TICKS_PER_ROTATION
            * moduleType.getSteerReduction() * 360;

    private static final double DRIVE_MOTOR_P = 1;
    private static final double DRIVE_MOTOR_D = 0.1;

    private static final double TURN_MOTOR_P = 0.2;
    private static final double TURN_MOTOR_D = 0.1;

    public final WPI_TalonFX driveMotor;
    public final WPI_TalonFX turnMotor;

    public SumedhsDatatouilleSwerveModule(int driveMotorPort, int turnMotorPort, String canbus) {
        driveMotor = new WPI_TalonFX(driveMotorPort, canbus);
        turnMotor = new WPI_TalonFX(turnMotorPort, canbus);

        TalonFXConfiguration driveMotorSettings = new TalonFXConfiguration();
        driveMotorSettings.supplyCurrLimit.currentLimit = moduleConfigs.getDriveCurrentLimit();
        driveMotorSettings.supplyCurrLimit.enable = true;
        driveMotorSettings.voltageCompSaturation = moduleConfigs.getNominalVoltage();

        driveMotor.configAllSettings(driveMotorSettings);
        driveMotor.setInverted(moduleType.isDriveInverted());
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.config_kP(0, DRIVE_MOTOR_P);
        driveMotor.config_kD(0, DRIVE_MOTOR_D);

        TalonFXConfiguration turnMotorSettings = new TalonFXConfiguration();
        turnMotorSettings.supplyCurrLimit.currentLimit = moduleConfigs.getSteerCurrentLimit();
        turnMotorSettings.supplyCurrLimit.enable = true;
        turnMotorSettings.voltageCompSaturation = moduleConfigs.getNominalVoltage();

        turnMotor.configAllSettings(turnMotorSettings);
        turnMotor.setInverted(moduleType.isSteerInverted());
        turnMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.enableVoltageCompensation(true);
        turnMotor.config_kP(0, TURN_MOTOR_P);
        turnMotor.config_kD(0, TURN_MOTOR_D);

        resetEncoder();
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getAngle());
        driveMotor.set(ControlMode.Velocity,
                state.speedMetersPerSecond / DRIVE_MOTOR_ENCODER_VELOCITY_TO_METERS_PER_SECOND);
        turnMotor.set(ControlMode.Position, state.angle.getDegrees() / TURN_MOTOR_ENCODER_TICKS_TO_DEGREE);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMeter(), getAngle());
    }

    public double getDriveVelocityMeter() {
        return driveMotor.getSelectedSensorVelocity() * DRIVE_MOTOR_ENCODER_VELOCITY_TO_METERS_PER_SECOND;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(turnMotor.getSelectedSensorPosition() / TURN_MOTOR_ENCODER_TICKS_TO_DEGREE);
    }

    public void resetEncoder() {
        driveMotor.setSelectedSensorPosition(0);
        turnMotor.setSelectedSensorPosition(0);
    }

    public void setToCoast() {
        driveMotor.setNeutralMode(NeutralMode.Coast);
        turnMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void setToBrake() {
        driveMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.setNeutralMode(NeutralMode.Brake);
    }

}
