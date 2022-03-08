package frc.team2412.robot.commands.shooter;

import static frc.team2412.robot.subsystem.ShooterSubsystem.ShooterConstants.MIN_HOOD_ANGLE;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;

public class ShooterMisfireCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final ShooterVisionSubsystem vision;

    /**
     * Subsystems: {@link ShooterSubsystem}, {@link ShooterVisionSubsystem}
     */
    public ShooterMisfireCommand() {
        this.shooter = ShooterSubsystem.instance;
        this.vision = ShooterVisionSubsystem.instance;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double yaw = vision.getYaw() + shooter.getTurretAngleBias();
        shooter.setHoodAngle(MIN_HOOD_ANGLE);
        shooter.setFlywheelRPM(4000);
        shooter.updateTurretAngle(yaw);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
