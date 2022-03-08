package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeInCommand extends SequentialCommandGroup {

    public IntakeInCommand() {
        addCommands(
                new IntakeExtendCommand(),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                        new IntakeMotorInCommand()));// ,
        // new IndexShootCommand(indexSubsystem)));
    }
}
