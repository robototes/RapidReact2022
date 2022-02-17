package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.index.IndexCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeInCommand extends SequentialCommandGroup {

    public IntakeInCommand(IndexSubsystem indexSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new IntakeExtendCommand(intakeSubsystem),
                new WaitCommand(0.1),
                new ParallelCommandGroup(
                        new IntakeMotorInCommand(intakeSubsystem),
                        new IndexCommand(indexSubsystem)));

    }
}
