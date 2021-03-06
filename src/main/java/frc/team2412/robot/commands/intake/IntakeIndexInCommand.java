package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeIndexInCommand extends ParallelCommandGroup {

    public IntakeIndexInCommand(IndexSubsystem indexSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(new IntakeSetInCommand(intakeSubsystem), new IndexShootCommand(indexSubsystem));
    }
}
