package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.*;

import java.util.function.BooleanSupplier;

import frc.team2412.robot.commands.drive.DriveCommand;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.DPadButton.Direction;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.team2412.robot.commands.climb.ClimbSetArmCommand;
import frc.team2412.robot.commands.climb.ExtendArmCommand;
import frc.team2412.robot.commands.climb.PostClimbUpComamnd;
import frc.team2412.robot.commands.climb.RetractArmCommand;
import frc.team2412.robot.commands.index.IndexCommand;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.intake.IntakeCommand;
import frc.team2412.robot.commands.intake.IntakeSetRetractCommand;
import frc.team2412.robot.commands.intake.SpitBallCommand;
import frc.team2412.robot.commands.shooter.ShooterHoodRPMCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;

@SuppressWarnings("unused")
public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;
        public static final int CODRIVER_CONTROLLER_PORT = 1;
    }

    public XboxController driveController;
    public XboxController codriverController;

    // climb
    public final Button climbArmUp;
    public final Button climbArmDown;
    public final Button climbArmDownManual;
    public final Button climbArmUpManual;

    public Controller shootPreset, climbPreset;

    // index

    // shooter
    public final Button shootButton;

    // intake
    public final Button[] intakeInButton;
    public final Button[] intakeSpitButton;
    public final Button intakeRetractButton;

    // drive
    public final Button resetDriveGyroButton;
    public final Button setPoseButton;

    public Subsystems subsystems;

    public Controls(Subsystems s) {

        subsystems = s;

        driveController = new XboxController(CONTROLLER_PORT);
        codriverController = new XboxController(CODRIVER_CONTROLLER_PORT);

        resetDriveGyroButton = driveController.getRightJoystickButton();
        setPoseButton = driveController.getStartButton(); // set pose button is practice bot only
        shootButton = driveController.getRightBumperButton();
        intakeInButton = new Button[] { driveController.getAButton(),
                driveController.getLeftTriggerAxis().getButton(0.1),
                driveController.getRightTriggerAxis().getButton(0.1),
        };

        intakeSpitButton = new Button[] { driveController.getBButton() };
        intakeRetractButton = driveController.getYButton();

        climbArmUp = codriverController.getStartButton();
        climbArmDown = codriverController.getBackButton();
        climbArmDownManual = codriverController.getRightBumperButton();
        climbArmUpManual = codriverController.getLeftBumperButton();

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
        climbArmDown.whenPressed(new RetractArmCommand(subsystems.climbSubsystem));
        climbArmUp.whenPressed(new ExtendArmCommand(subsystems.climbSubsystem));

        climbArmDownManual.whileHeld(new ClimbSetArmCommand(subsystems.climbSubsystem, -0.4));
        climbArmUpManual.whileHeld(new ClimbSetArmCommand(subsystems.climbSubsystem, 0.4));
    }

    public void bindPostClimbControls() {
        climbArmUp.and(climbArmDown).and(climbArmDownManual).and(climbArmUpManual)
                .whenActive(new PostClimbUpComamnd(subsystems.postClimbSubsystem));
    }

    public void bindDriveControls() {
        subsystems.drivebaseSubsystem.setDefaultCommand(new DriveCommand(subsystems.drivebaseSubsystem,
                driveController.getLeftYAxis(), driveController.getLeftXAxis(),
                driveController.getRightXAxis()));
        resetDriveGyroButton.whenPressed(() -> subsystems.drivebaseSubsystem.resetGyroAngle(Rotation2.ZERO));

        if (!Robot.getInstance().isCompetition())
            setPoseButton.whenPressed(() -> subsystems.drivebaseSubsystem.setPose());
    }

    public void bindIndexControls() {
        if (subsystems.intakeSubsystem != null) {
            subsystems.indexSubsystem
                    .setDefaultCommand(new IndexCommand(subsystems.indexSubsystem, subsystems.intakeSubsystem));

            subsystems.intakeSubsystem
                    .setDefaultCommand(new IntakeCommand(subsystems.intakeSubsystem, subsystems.indexSubsystem));

        }

        shootButton.whileHeld(new IndexShootCommand(subsystems.indexSubsystem, subsystems.targetLocalizer));
    }

    public void bindIntakeControls() {

        for (Button b : intakeSpitButton)
            b.whileHeld(new SpitBallCommand(subsystems.indexSubsystem, subsystems.intakeSubsystem));
        intakeRetractButton.whenPressed(new IntakeSetRetractCommand(subsystems.intakeSubsystem));
    }

    public void bindShooterControls() {
        if (!Subsystems.SubsystemConstants.SHOOTER_TESTING) {
            BooleanSupplier b = driveController.getDPadButton(Direction.UP)::get;

            driveController.getDPadButton(Direction.DOWN).whenPressed(
                    new ShooterHoodRPMCommand(subsystems.shooterSubsystem, 1400, 35).withInterrupt(b)
                            .alongWith(new InstantCommand(() -> subsystems.shooterSubsystem.setTurretAngle(-90))));

            driveController.getDPadButton(Direction.LEFT).whenPressed(
                    new ShooterHoodRPMCommand(subsystems.shooterSubsystem, 3092, 0).withInterrupt(b)
                            .alongWith(new InstantCommand(() -> subsystems.shooterSubsystem.setTurretAngle(-90))));

            driveController.getDPadButton(Direction.RIGHT).whenPressed(
                    new ShooterHoodRPMCommand(subsystems.shooterSubsystem, 0, 0).withInterrupt(b)
                            .alongWith(new InstantCommand(() -> subsystems.shooterSubsystem.setTurretAngle(0))));

            if (subsystems.drivebaseSubsystem != null) {
                subsystems.shooterSubsystem.setDefaultCommand(
                        new ShooterTargetCommand(subsystems.shooterSubsystem, subsystems.targetLocalizer,
                                driveController.getLeftBumperButton()::get));

            }

        }
    }
}
