package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.CONTROLLER_PORT;

import java.util.function.BooleanSupplier;

import frc.team2412.robot.commands.drive.DriveCommand;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.DPadButton.Direction;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.team2412.robot.commands.climb.ExtendArmCommand;
import frc.team2412.robot.commands.climb.RetractArmFullyCommand;
import frc.team2412.robot.commands.index.IndexCommand;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.intake.IntakeCommand;
import frc.team2412.robot.commands.intake.IntakeSetRetractCommand;
import frc.team2412.robot.commands.intake.SpitBallCommand;
import frc.team2412.robot.commands.shooter.ShooterHoodRPMCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.commands.shooter.ShooterTurretSetAngleCommand;

@SuppressWarnings("unused")
public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;
        public static final int CODRIVER_CONTROLLER_PORT = 1;
    }

    public XboxController driveController;

    // climb
    public final Button climbFixedArmUp;
    public final Button climbFixedArmDown;

    public Controller shootPreset, climbPreset;
    public XboxController codriverController;

    // index

    // shooter
    public final Button shootButton;

    // intake
    public final Button[] intakeInButton;
    public final Button[] intakeSpitButton;
    public final Button intakeRetractButton;

    // drive
    public final Button resetDriveGyroButton;

    public Subsystems subsystems;

    public Controls(Subsystems s) {

        subsystems = s;

        driveController = new XboxController(CONTROLLER_PORT);

        resetDriveGyroButton = driveController.getRightJoystickButton();
        shootButton = driveController.getRightBumperButton();

        intakeInButton = new Button[] { driveController.getAButton(),
                driveController.getLeftTriggerAxis().getButton(0.1),
                driveController.getRightTriggerAxis().getButton(0.1),
        };

        intakeSpitButton = new Button[] { driveController.getBButton() };
        intakeRetractButton = driveController.getYButton();
        climbFixedArmUp = driveController.getStartButton();
        climbFixedArmDown = driveController.getBackButton();

        boolean comp = Robot.getInstance().isCompetition();

        if (subsystems.drivebaseSubsystem != null) {
            bindDriveControls();
        }

        if (subsystems.climbSubsystem != null) {
            bindClimbControls();
        }

        if (subsystems.indexSubsystem != null) {
            bindIndexControls();
        }

        if (subsystems.intakeSubsystem != null && subsystems.indexSubsystem != null) {
            bindIntakeControls();
        }

        if (subsystems.shooterSubsystem != null) {
            bindShooterControls();
        }
    }

    public void bindClimbControls() {
        climbFixedArmDown.whenPressed(new RetractArmFullyCommand(subsystems.climbSubsystem));
        climbFixedArmUp.whenPressed(new ExtendArmCommand(subsystems.climbSubsystem));
    }

    public void bindDriveControls() {
        subsystems.drivebaseSubsystem.setDefaultCommand(new DriveCommand(subsystems.drivebaseSubsystem,
                driveController.getLeftYAxis(), driveController.getLeftXAxis(),
                driveController.getRightXAxis()));
        resetDriveGyroButton.whenPressed(() -> subsystems.drivebaseSubsystem.resetGyroAngle(Rotation2.ZERO));
    }

    public void bindIndexControls() {
        subsystems.indexSubsystem
                .setDefaultCommand(new IndexCommand(subsystems.indexSubsystem, subsystems.intakeSubsystem));

        shootButton.whileHeld(new IndexShootCommand(subsystems.indexSubsystem));
    }

    public void bindIntakeControls() {
        subsystems.intakeSubsystem
                .setDefaultCommand(new IntakeCommand(subsystems.intakeSubsystem, subsystems.indexSubsystem));

        for (Button b : intakeSpitButton)
            b.whileHeld(new SpitBallCommand(subsystems.indexSubsystem, subsystems.intakeSubsystem));
        intakeRetractButton.whenPressed(new IntakeSetRetractCommand(subsystems.intakeSubsystem));
    }

    public void bindShooterControls() {
        if (!Subsystems.SubsystemConstants.SHOOTER_TESTING) {
            BooleanSupplier b = driveController.getDPadButton(Direction.UP)::get;

            driveController.getDPadButton(Direction.DOWN).whenPressed(
                    new ShooterHoodRPMCommand(subsystems.shooterSubsystem, 1000, 35).withInterrupt(b));
            driveController.getDPadButton(Direction.LEFT).whenPressed(
                    new ShooterHoodRPMCommand(subsystems.shooterSubsystem, 2100, 13.7).withInterrupt(b));

            driveController.getDPadButton(Direction.RIGHT).whenPressed(
                    new ShooterHoodRPMCommand(subsystems.shooterSubsystem, 0, 0)
                            .andThen(new ShooterTurretSetAngleCommand(subsystems.shooterSubsystem, -90))
                            .withInterrupt(b));

            if (subsystems.drivebaseSubsystem != null) {
                subsystems.shooterSubsystem.setDefaultCommand(
                        new ShooterTargetCommand(subsystems.shooterSubsystem, subsystems.targetLocalizer,
                                driveController.getLeftBumperButton()::get));

            }

        }
    }
}
