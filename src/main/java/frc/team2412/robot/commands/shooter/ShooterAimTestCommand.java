package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterAimTestCommand extends CommandBase {
    private ShooterSubsystem shooter;

    public ShooterAimTestCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setFlywheelRPM(shooter.getFlywheelTestRPM());
        shooter.setHoodAngle(shooter.getHoodTestAngle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
