package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.commands.intake.IntakeIndexInCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.TargetLocalizer;

public class TestingShootCommand extends ParallelCommandGroup {

    public TestingShootCommand(IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem,
            TargetLocalizer localizer, IntakeSubsystem intakeSubsystem) {
        addCommands(new IntakeIndexInCommand(indexSubsystem, intakeSubsystem),
                new ShooterTargetCommand(shooterSubsystem, localizer));
    }
}
