package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterHoodMotorStopCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    public ShooterHoodMotorStopCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.stopHoodMotor();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
