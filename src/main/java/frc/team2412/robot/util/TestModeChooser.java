package frc.team2412.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.climb.ClimbTestCommand;
import frc.team2412.robot.commands.intake.IntakeInCommand;

public class TestModeChooser {

    private SendableChooser<TestMode> testModeChooser = new SendableChooser<>();

    public TestModeChooser() {
        testModeChooser.setDefaultOption("Climb test", TestMode.CLIMB);
        testModeChooser.addOption("Index test", TestMode.INDEX);
        testModeChooser.addOption("Intake test", TestMode.INTAKE);
        testModeChooser.addOption("Shooter test", TestMode.SHOOTER);

        ShuffleboardTab testTab = Shuffleboard.getTab("Tests");

        testTab.add("Choose Auto Mode", testModeChooser)
                .withPosition(0, 0)
                .withSize(2, 1);
    }

    public CommandBase getCommand(Subsystems subsystems) {
        switch (testModeChooser.getSelected()) {
            case CLIMB:
                System.out.println("Climb test command chosen");
                return new ClimbTestCommand(subsystems.climbSubsystem);
            case INDEX:
                System.out.println("Index test command chosen");
                return new IntakeInCommand(subsystems.intakeSubsystem);
            case INTAKE:
                System.out.println("Intake test command chosen");
                // return new IntakeTestCommand(subsystems);
                return new IntakeInCommand(subsystems.intakeSubsystem);
            case SHOOTER:
                System.out.println("Shooter test command chosen");
                // return new ShooterTestCommand(subsystems);
                return new SequentialCommandGroup();
            default:
                System.out.println("No command chosen");
                return new SequentialCommandGroup();
        }
    }

    private enum TestMode {
        CLIMB, INDEX, INTAKE, SHOOTER
    }
}
