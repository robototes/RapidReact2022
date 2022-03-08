package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.ClimbSubsystem;

public class ClimbStageChooserCommand extends CommandBase {

    private ClimbSubsystem subsystem;

    public ClimbStageChooserCommand(ClimbSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void initialize() {
        switch (subsystem.getAutoClimbState()) {
            case GROUND_MID:
                new ClimbGroundToMidCommand(subsystem).schedule();
                break;
            case MID_HIGH:
                new ClimbMidToHighCommand(subsystem).schedule();
                break;
            case HIGH_TRAV:
                new ClimbHighToTravCommand(subsystem).schedule();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
