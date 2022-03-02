package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.CODRIVER_CONTROLLER_PORT;
import static frc.team2412.robot.Controls.ControlConstants.CONTROLLER_PORT;
import static frc.team2412.robot.Subsystems.SubsystemConstants.CLIMB_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.DRIVE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INDEX_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INTAKE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.SHOOTER_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.SHOOTER_VISION_ENABLED;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.DPadButton.Direction;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.team2412.robot.commands.climb.ExtendFixedHookCommand;
import frc.team2412.robot.commands.climb.FullExtendFixedHookCommand;
import frc.team2412.robot.commands.climb.FullRetractFixedHookCommand;
import frc.team2412.robot.commands.climb.RetractFixedHookCommand;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.intake.IntakeExtendCommand;
import frc.team2412.robot.commands.intake.IntakeInCommand;
import frc.team2412.robot.commands.intake.IntakeOutCommand;
import frc.team2412.robot.commands.intake.IntakeRetractCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;

public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;
        public static final int CODRIVER_CONTROLLER_PORT = 1;
    }

    public XboxController driveController;
    public XboxController codriverController;

    // climb
    public final Button climbFixedArmUp;
    public final Button climbFixedArmFullUp;
    public final Button climbFixedArmDown;
    public final Button climbFixedArmFullDown;

    // shooter
    public final Button shootButton;
    public final Button hoodUpButton;
    public final Button hoodDownButton;
    public final Button turretLeftButton;
    public final Button turretRightButton;

    // intake
    public final Button intakeInButton;
    public final Button intakeSpitButton;
    public final Button intakeRetractButton;

    // drive
    public final Button resetDriveGyroButton;

    public Subsystems subsystems;

    public Controls(Subsystems s) {

        subsystems = s;

        driveController = new XboxController(CONTROLLER_PORT);
        codriverController = new XboxController(CODRIVER_CONTROLLER_PORT);

        climbFixedArmUp = codriverController.getLeftBumperButton();
        climbFixedArmFullUp = codriverController.getBButton();
        climbFixedArmDown = codriverController.getRightBumperButton();
        climbFixedArmFullDown = codriverController.getXButton();

        resetDriveGyroButton = driveController.getRightJoystickButton();

        intakeInButton = driveController.getRightBumperButton();
        intakeSpitButton = driveController.getAButton();
        intakeRetractButton = driveController.getBButton();

        shootButton = driveController.getLeftBumperButton();
        hoodUpButton = driveController.getDPadButton(Direction.UP);
        hoodDownButton = driveController.getDPadButton(Direction.DOWN);
        turretLeftButton = driveController.getDPadButton(Direction.LEFT);
        turretRightButton = driveController.getDPadButton(Direction.RIGHT);

        if (CLIMB_ENABLED) {
            bindClimbControls();
        }
        if (DRIVE_ENABLED) {
            bindDriveControls();
        }
        if (INDEX_ENABLED)
            bindIndexControls();

        if (INTAKE_ENABLED) {
            bindIntakeControls();
        }

        if (SHOOTER_ENABLED && SHOOTER_VISION_ENABLED) {
            bindShooterControls();
        }
    }

    public void bindClimbControls() {
        climbFixedArmUp.whileHeld(new ExtendFixedHookCommand(subsystems.climbSubsystem));
        climbFixedArmDown.whileHeld(new RetractFixedHookCommand(subsystems.climbSubsystem));
        climbFixedArmFullUp.whenPressed(new FullExtendFixedHookCommand(subsystems.climbSubsystem));
        climbFixedArmFullDown.whenPressed(new FullRetractFixedHookCommand(subsystems.climbSubsystem));
    }

    public void bindDriveControls() {
        resetDriveGyroButton.whenPressed(() -> {
            subsystems.drivebaseSubsystem.resetGyroAngle(Rotation2.ZERO);
        });
    }

    public void bindIndexControls() {
        if (SHOOTER_ENABLED && SHOOTER_VISION_ENABLED && INDEX_ENABLED) {
            shootButton.whenPressed(new IndexShootCommand(subsystems.indexSubsystem));
        }
    }

    public void bindIntakeControls() {
        intakeInButton.whenPressed(new IntakeExtendCommand(subsystems.intakeSubsystem)
                .andThen(new IntakeInCommand(subsystems.intakeSubsystem)));
        intakeRetractButton.whenPressed(new IntakeRetractCommand(subsystems.intakeSubsystem));
        intakeSpitButton.whenPressed(new IntakeOutCommand(subsystems.intakeSubsystem));

    }

    public void bindShooterControls() {
        subsystems.shooterSubsystem.setDefaultCommand(
                new ShooterTargetCommand(subsystems.shooterSubsystem, subsystems.shooterVisionSubsystem));

    }
}
