package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeExtendCommand extends CommandBase {
    private final IntakeSubsystem subsystem;

    public IntakeExtendCommand(IntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void execute() {
        subsystem.intakeExtend();
        System.out.println("1");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
