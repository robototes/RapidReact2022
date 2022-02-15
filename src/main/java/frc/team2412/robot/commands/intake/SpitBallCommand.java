package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.commands.index.IndexSpitCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class SpitBallCommand extends ParallelCommandGroup {
    private final IndexSubsystem indexSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public SpitBallCommand(IndexSubsystem indexSubsystem, IntakeSubsystem intakeSubsystem) {
        this.indexSubsystem = indexSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(indexSubsystem, intakeSubsystem);
        addCommands(
                new IntakeExtendCommand(intakeSubsystem),
                new IntakeOutCommand(intakeSubsystem),
                new IndexSpitCommand(indexSubsystem));
    }

}
