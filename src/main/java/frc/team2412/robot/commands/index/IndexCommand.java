package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IndexCommand extends CommandBase {
    private IndexSubsystem indexSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public IndexCommand(IndexSubsystem subsystem, IntakeSubsystem intakeSubsystem) {
        this.indexSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if(!intakeSubsystem.isIntakeExtended()) {
            indexSubsystem.feederMotorStop();
            indexSubsystem.ingestMotorStop();
            return;
        }

        if (indexSubsystem.hasCargo()) {
            indexSubsystem.feederMotorStop();
        } else {
            indexSubsystem.feederMotorIn();
        }

        if(intakeSubsystem.hasCargo()){
            indexSubsystem.ingestMotorStop();
        } else {
            indexSubsystem.ingestMotorIn();
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
