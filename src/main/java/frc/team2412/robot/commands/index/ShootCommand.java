package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;

public class ShootCommand extends ParallelCommandGroup {

    public ShootCommand() {
        addCommands(new IndexShootCommand(), new ShooterTargetCommand());
    }
}
