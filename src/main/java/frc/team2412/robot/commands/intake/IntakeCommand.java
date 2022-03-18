package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    private IndexSubsystem indexSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexSubsystem = indexSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (!(indexSubsystem.hasCargo() && intakeSubsystem.hasCargo())) {
            intakeSubsystem.intakeIn();
            indexSubsystem.ingestMotorIn();
        } else {
            intakeSubsystem.intakeStop();
            indexSubsystem.ingestMotorStop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
