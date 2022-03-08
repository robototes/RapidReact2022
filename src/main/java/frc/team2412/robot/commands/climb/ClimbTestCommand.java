package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimbTestCommand extends SequentialCommandGroup {
    public ClimbTestCommand() {
        addCommands(new ExtendAngledHookCommand(),
                new RetractAngledHookCommand(),
                new UnangleClimbHookCommand(),
                new NeutralClimbHookCommand(),
                new AngleClimbHookCommand(),
                new ExtendFixedHookCommand(),
                new RetractFixedHookCommand());
    }
}
