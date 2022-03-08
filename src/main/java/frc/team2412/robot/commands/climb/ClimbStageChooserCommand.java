package frc.team2412.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team2412.robot.subsystem.ClimbSubsystem;

public class ClimbStageChooserCommand extends CommandBase {
    private ClimbSubsystem subsystem;

    /**
     * Subsystems: {@link ClimbSubsystem}
     */
    public ClimbStageChooserCommand() {
        this.subsystem = ClimbSubsystem.instance;
    }

    @Override
    public void initialize() {
        switch (subsystem.getAutoClimbState()) {
            case GROUND_MID:
                new ClimbGroundToMidCommand().schedule();
                break;
            case MID_HIGH:
                new ClimbMidToHighCommand().schedule();
                break;
            case HIGH_TRAV:
                new ClimbHighToTravCommand().schedule();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
