package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.commands.intake.IntakeInCommand;

public class FullShootCommand extends ParallelCommandGroup {
    public FullShootCommand() {
        addCommands(new ShooterTargetCommand(), new IntakeInCommand());
    }
}
