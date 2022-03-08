package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeInCommand extends SequentialCommandGroup {
    /**
     * Subsystems: {@link IndexSubsystem}, {@link IntakeSubsystem}
     */
    public IntakeInCommand() {
        addCommands(
                new IntakeExtendCommand(),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                        new IntakeMotorInCommand()));// ,
        // new IndexShootCommand(indexSubsystem)));
    }
}
