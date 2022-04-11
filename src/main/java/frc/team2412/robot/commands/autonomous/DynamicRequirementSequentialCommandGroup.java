package frc.team2412.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class DynamicRequirementSequentialCommandGroup extends SequentialCommandGroup {
    public DynamicRequirementSequentialCommandGroup(Command... commands) {
        addCommands2(commands);
    }

    public void addCommands2(Command... commands) {
        Command[] newCommands = new Command[commands.length * 2];
        for (int i = 0; i < commands.length; i++) {
            newCommands[2 * i] = new ScheduleCommand(commands[i]);
            newCommands[2 * i + 1] = new WaitUntilCommand(commands[i]::isFinished);
        }
        addCommands(newCommands);
    }
}
