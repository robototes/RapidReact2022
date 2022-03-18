package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.commands.intake.IntakeIndexInCommand;
import frc.team2412.robot.subsystem.*;

public class FullShootCommand extends ParallelCommandGroup {
    public FullShootCommand(ShooterSubsystem shooter, TargetLocalizer localizer, IntakeSubsystem intake,
            IndexSubsystem index) {
        addCommands(new ShooterTargetCommand(shooter, localizer), new IntakeIndexInCommand(index, intake));
    }
}
