package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.index.IndexSpitCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class SpitBallCommand extends SequentialCommandGroup {

    public SpitBallCommand(IndexSubsystem indexSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new IntakeExtendCommand(intakeSubsystem),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                        new IntakeMotorOutCommand(intakeSubsystem),
                        new IndexSpitCommand(indexSubsystem)));
    }
}
