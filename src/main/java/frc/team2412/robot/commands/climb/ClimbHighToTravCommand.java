package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ClimbSubsystem;

public class ClimbHighToTravCommand extends CommandBase {

    ClimbSubsystem subsystem;

    public ClimbHighToTravCommand(ClimbSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        new AngleClimbHookCommand(subsystem)
                .andThen(new ExtendAngledHookCommand(subsystem))
                .andThen(new RetractAngledHookCommand(subsystem));
    }

}
