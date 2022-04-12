package frc.team2412.robot.commands.shooter;

import static frc.team2412.robot.subsystem.ShooterSubsystem.ShooterConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.TargetLocalizer;
import frc.team2412.robot.util.ShooterDataDistancePoint;

public class ShooterTargetCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final TargetLocalizer localizer;
    private BooleanSupplier turretDisable;
    private DoubleSupplier turretIdlePosition;

    private double turretAngle = 0;

    public ShooterTargetCommand(ShooterSubsystem shooter, TargetLocalizer localizer) {
        this(shooter, localizer, () -> false);
    }

    public ShooterTargetCommand(ShooterSubsystem shooter, TargetLocalizer localizer, BooleanSupplier turretButton) {
        this(shooter, localizer, turretButton, () -> 0);
    }

    public ShooterTargetCommand(ShooterSubsystem shooter, TargetLocalizer localizer, BooleanSupplier turretButton,
            DoubleSupplier turretAngle) {
        this.shooter = shooter;
        this.localizer = localizer;
        turretDisable = turretButton;
        turretIdlePosition = turretAngle;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        localizer.limelightOn();
    }

    public enum TurretState {
        WRAP_LEFT, WRAP_RIGHT, STOPPED, TRACKING;
    }

    TurretState state;

    @Override
    public void execute() {
        if (!localizer.hasTarget())
            return;

        if (ShooterConstants.DATA_POINTS != null && localizer.getAdjustedDistance() < 280) {
            ShooterDataDistancePoint shooterData = ShooterConstants.DATA_POINTS

                    .getInterpolated(localizer.getAdjustedDistance());

            // System.out.println("Limelight distance: " + localizer.getDistance());
            // System.out.println("Localizer distance" + localizer.getAdjustedDistance());

            // System.out.println(shooterData);

            shooter.setHoodAngle(shooterData.getAngle());
            shooter.setFlywheelRPM(shooterData.getRPM());

            // System.out.println("Actual shooter RPM : " + shooter.getFlywheelRPM());
            // System.out.println("Actual hood angle: " + shooter.getHoodAngle());
        }

        if (turretDisable.getAsBoolean())
            state = TurretState.STOPPED;
        else if (turretAngle < ShooterConstants.LEFT_WRAP_THRESHOLD)
            state = TurretState.WRAP_LEFT;
        else if (turretAngle > ShooterConstants.RIGHT_WRAP_THRESHOLD)
            state = TurretState.WRAP_RIGHT;
        else if (turretAngle > ShooterConstants.LEFT_WRAP && turretAngle < ShooterConstants.RIGHT_WRAP)
            state = TurretState.TRACKING;

        switch (state) {
            case STOPPED:
                turretAngle = turretIdlePosition.getAsDouble();
                break;
            case WRAP_LEFT:
                turretAngle = ShooterConstants.RIGHT_WRAP;
                if (shooter.isTurretAtAngle(ShooterConstants.RIGHT_WRAP)
                        || shooter.getTurretAngle() >= ShooterConstants.RIGHT_WRAP)
                    state = TurretState.TRACKING;
                break;
            case WRAP_RIGHT:
                turretAngle = ShooterConstants.LEFT_WRAP;
                if (shooter.isTurretAtAngle(ShooterConstants.LEFT_WRAP)
                        || shooter.getTurretAngle() <= ShooterConstants.LEFT_WRAP)
                    state = TurretState.TRACKING;
                break;
            case TRACKING:
                turretAngle = shooter.getTurretAngle() + localizer.getTargetYaw();
                break;
        }

        double localizerTurretAdjustment = state == TurretState.TRACKING ? localizer.yawAdjustment() : 0;

        // System.out.println("Localizer turret adjustment: " + localizerTurretAdjustment);

        turretAngle = turretAngle + localizerTurretAdjustment;
        // System.out.println("turret angle : " + turretAngle);

        shooter.setTurretAngle(turretAngle);

        // System.out.println("Actual turret angle : " + shooter.getTurretAngle());
    }

    @Override
    public void end(boolean interrupted) {
        localizer.limelightOff();
    }

    public static class TurretManager {
        public final ShooterTargetCommand shooterTargetCommand;
        public double idle;
        public boolean enabled;

        public TurretManager(ShooterSubsystem shooterSubsystem, TargetLocalizer localizer) {
            shooterTargetCommand = new ShooterTargetCommand(shooterSubsystem, localizer, this::getTurretDisable,
                    this::getIdlePosition);
            idle = 0;
            enabled = false;
        }

        // public TurretManager(ShooterTargetCommand targetCommand) {
        // shooterTargetCommand = targetCommand;
        // shooterTargetCommand.turretIdlePosition = this::getIdlePosition;
        // shooterTargetCommand.turretEnable = this::getTurretEnable;
        // idle = 0;
        // enabled = false;
        // }

        public ShooterTargetCommand getCommand() {
            return shooterTargetCommand;
        }

        public ScheduleCommand scheduleCommand() {
            return new ScheduleCommand(shooterTargetCommand);
        }

        public InstantCommand cancelCommand() {
            return new InstantCommand(shooterTargetCommand::cancel);
        }

        public InstantCommand enableAt(double idleTurretAngle) {
            return manageTurret(true, idleTurretAngle);
        }

        public InstantCommand disableAt(double idleTurretAngle) {
            return manageTurret(false, idleTurretAngle);
        }

        public InstantCommand enable() {
            return enableAt(idle);
        }

        public InstantCommand disable() {
            return disableAt(idle);
        }

        public InstantCommand changeIdle(double id) {
            return manageTurret(enabled, id);
        }

        public InstantCommand manageTurret(boolean enable, double id) {
            return new InstantCommand(() -> {
                enabled = enable;
                idle = id;
            });
        }

        public double getIdlePosition() {
            return idle;
        }

        public boolean getTurretDisable() {
            return !enabled;
        }
    }
}
