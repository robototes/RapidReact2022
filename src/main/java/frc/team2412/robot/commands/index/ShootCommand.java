package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.TargetLocalizer;

public class ShootCommand extends ParallelCommandGroup {

    public ShootCommand(IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem,
            TargetLocalizer localizer) {
        addCommands(new IndexShootCommand(indexSubsystem),
                new ShooterTargetCommand(shooterSubsystem, localizer));
    }
}
