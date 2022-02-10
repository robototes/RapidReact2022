package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexCommand extends SequentialCommandGroup {
    private final IndexSubsystem subsystem;

    public IndexCommand(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void initialize() {
        // move the ball into the first sensor
        addCommands(new IndexMoveBallToFirstSensor(subsystem));

        // if both spaces are open
        if (!subsystem.feederSensorHasBallIn()) {
            // move ball from first to seconds
            addCommands(new IndexMoveBallToSecondSensor(subsystem));
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
