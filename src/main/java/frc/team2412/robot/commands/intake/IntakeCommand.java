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
        if(!intakeSubsystem.isIntakeExtended()){
            indexSubsystem.feederMotorStop();
            indexSubsystem.ingestMotorStop();
            intakeSubsystem.intakeStop();
            return;
        }

        if (indexSubsystem.hasCargo() && intakeSubsystem.hasCargo()) {
            intakeSubsystem.intakeStop();
            indexSubsystem.ingestMotorStop();
        } else {
            intakeSubsystem.intakeIn();
            indexSubsystem.ingestMotorIn();

        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
