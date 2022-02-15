package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.commands.index.IndexCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeBallCommand extends ParallelCommandGroup {
    private final IndexSubsystem indexSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public IntakeBallCommand(IndexSubsystem indexSubsystem, IntakeSubsystem intakeSubsystem) {
        this.indexSubsystem = indexSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addCommands(
                new IntakeExtendCommand(intakeSubsystem),
                new IntakeInCommand(intakeSubsystem),
                new IndexCommand(indexSubsystem));
    }
}
