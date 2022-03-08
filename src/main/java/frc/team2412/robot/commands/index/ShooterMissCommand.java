package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.commands.shooter.ShooterMisfireCommand;

public class ShooterMissCommand extends ParallelCommandGroup {

    public ShooterMissCommand() {
        addCommands(new IndexShootCommand(), new ShooterMisfireCommand());
    }
}
