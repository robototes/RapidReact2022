package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.subsystem.ClimbSubsystem;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class DiagnosticRobotCommandGroup extends SequentialCommandGroup {
    ShuffleboardTab tab = Shuffleboard.getTab("Self Diagnostic");
    NetworkTableEntry intakeStatus, shooterStatus, indexStatus, climbStatus;

    public DiagnosticRobotCommandGroup(
            IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem,
            IndexSubsystem indexSubsystem,
            ClimbSubsystem climbSubsystem) {
        /*
         * The Shuffleboard having issue of
         * cannot properly display more than 3 rows of entry in one column
         */
        indexStatus = tab.add("Index Status", "Waiting").withPosition(0, 0).withSize(1, 1).getEntry();
        shooterStatus = tab.add("Shooter Status", "Waiting").withPosition(1, 0).withSize(1, 1).getEntry();
        intakeStatus = tab.add("Intake Status", "Waiting").withPosition(2, 0).withSize(1, 1).getEntry();
        climbStatus = tab.add("Climb Status", "Waiting").withPosition(3, 0).withSize(1, 1).getEntry();
        addRequirements(intakeSubsystem, shooterSubsystem, indexSubsystem, climbSubsystem);
        /*
        addCommands(
                new InstantCommand(() -> intakeStatus.setString("In Progress")),
                new DiagnosticIntakeCommandGroup(intakeSubsystem),
                new InstantCommand(() -> intakeStatus.setString("Finished")),
                new WaitCommand(5),
                new InstantCommand(() -> shooterStatus.setString("In Progress")),
                new DiagnosticShooterCommandGroup(shooterSubsystem),
                new InstantCommand(() -> shooterStatus.setString("Finished")),
                new WaitCommand(5),
                new InstantCommand(() -> indexStatus.setString("In Progress")),
                new DiagnosticIndexCommandGroup(indexSubsystem),
                new InstantCommand(() -> indexStatus.setString("Finished")),
                new WaitCommand(5),
                new InstantCommand(() -> climbStatus.setString("In Progress")),
                new DiagnosticClimbCommandGroup(climbSubsystem),
                new InstantCommand(() -> climbStatus.setString("Finished")));
                */
    }

    @Override
    public String getName() {
        return "DiagnosticRobotCommandGroup";
    }
}
