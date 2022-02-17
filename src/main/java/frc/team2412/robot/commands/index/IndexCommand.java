package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexCommand extends SequentialCommandGroup {

    public IndexCommand(IndexSubsystem subsystem) {
        if (!subsystem.feederSensorHasBallIn()) {
            addCommands(new IndexMoveBallToFirstSensor(subsystem), new IndexMoveBallToSecondSensor(subsystem));
        } else {
            addCommands(new IndexMoveBallToFirstSensor(subsystem));
        }
    }

}
