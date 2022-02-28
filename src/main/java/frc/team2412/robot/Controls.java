package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.CONTROLLER_PORT;
import static frc.team2412.robot.Subsystems.SubsystemConstants.CLIMB_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.DRIVE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INDEX_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.INTAKE_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.SHOOTER_ENABLED;
import static frc.team2412.robot.Subsystems.SubsystemConstants.SHOOTER_VISION_ENABLED;

import frc.team2412.robot.util.controller.CompoundController;
import frc.team2412.robot.util.controller.MultiController;
import frc.team2412.robot.util.controller.MultiController.Controllers;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.DPadButton.Direction;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.intake.IntakeInCommand;
import frc.team2412.robot.commands.intake.IntakeRetractCommand;
import frc.team2412.robot.commands.intake.SpitBallCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;

public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;
    }

    public CompoundController<Controllers, XboxController> driverController;

    public Controller primaryController;
    public Controller secondaryController;
    public Controller combinedController;


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

    // climb
    public final Button fixedArmUpManualButton;
    public final Button fixedArmDownManualButton;
    public final Button dynamicArmUpManualButton;
    public final Button dynamicArmDownManualButton;

    public final Button fixedArmUpButton;
    public final Button fixedArmDownButton;
    public final Button dynamicArmUpButton;
    public final Button dynamicArmDownButton;

    public final Button rungClimbButton;

    public Subsystems subsystems;

    public Controls(Subsystems s) {
        subsystems = s;

        driverController = CompoundController.of(CONTROLLER_PORT, Controllers.PRIMARY, Controllers.SECONDARY, Controllers.UNIVERSAL);

        primaryController = driverController.getPreset(Controllers.PRIMARY);
        secondaryController = driverController.getPreset(Controllers.SECONDARY);
        combinedController = driverController.getPreset(Controllers.UNIVERSAL);

        fixedArmUpManualButton = secondaryController.getDPadButton(Direction.UP);
        fixedArmDownManualButton = secondaryController.getDPadButton(Direction.DOWN);
        dynamicArmUpManualButton = secondaryController.getDPadButton(Direction.LEFT);
        dynamicArmDownManualButton = secondaryController.getDPadButton(Direction.RIGHT);

        fixedArmUpButton = secondaryController.getXButton();
        fixedArmDownButton = secondaryController.getYButton();
        dynamicArmUpButton = secondaryController.getAButton();
        dynamicArmDownButton = secondaryController.getBButton();

        rungClimbButton = secondaryController.getRightBumperButton();

        resetDriveGyroButton = primaryController.getRightJoystickButton();

        intakeInButton = primaryController.getRightBumperButton();
        intakeSpitButton = primaryController.getAButton();
        intakeRetractButton = primaryController.getBButton();

        shootButton = primaryController.getLeftBumperButton();
        hoodUpButton = primaryController.getDPadButton(Direction.UP);
        hoodDownButton = primaryController.getDPadButton(Direction.DOWN);
        turretLeftButton = primaryController.getDPadButton(Direction.LEFT);
        turretRightButton = primaryController.getDPadButton(Direction.RIGHT);

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

    // TODO these yay
    public void bindClimbControls() {

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
        intakeInButton.whenPressed(new IntakeInCommand(subsystems.indexSubsystem, subsystems.intakeSubsystem));
        intakeSpitButton.whenPressed(new SpitBallCommand(subsystems.indexSubsystem, subsystems.intakeSubsystem));
        intakeRetractButton.whenPressed(new IntakeRetractCommand(subsystems.intakeSubsystem));
    }

    public void bindShooterControls() {
        if (!Subsystems.SubsystemConstants.SHOOTER_TESTING) {
            subsystems.shooterSubsystem.setDefaultCommand(
                    new ShooterTargetCommand(subsystems.shooterSubsystem, subsystems.shooterVisionSubsystem));
        }

    }
}
