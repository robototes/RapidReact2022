package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.commands.index.IndexSpitCommand;

public class SpitBallCommand extends ParallelCommandGroup {
    public SpitBallCommand() {
        addCommands(new IntakeMotorOutCommand(), new IndexSpitCommand());
    }
}
