package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.CONTROLLER_PORT;
import static frc.team2412.robot.Subsystems.SubsystemConstants.*;
import static frc.team2412.robot.util.controller.MultiController.Controllers.*;

import frc.team2412.robot.util.controller.CompoundController;
import frc.team2412.robot.util.controller.MultiController;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.DPadButton.Direction;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.team2412.robot.commands.index.IndexShootCommand;
import frc.team2412.robot.commands.intake.IntakeExtendCommand;
import frc.team2412.robot.commands.intake.IntakeMotorInCommand;
import frc.team2412.robot.commands.intake.IntakeMotorOutCommand;
import frc.team2412.robot.commands.intake.IntakeRetractCommand;
import frc.team2412.robot.commands.shooter.ShooterHoodSetConstantAngleCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.commands.shooter.ShooterTurretSetAngleCommand;

@SuppressWarnings("unused")
public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;
        public static final int CODRIVER_CONTROLLER_PORT = 1;
    }

    public CompoundController<MultiController.Controllers, XboxController> driveController;

    public Controller shootPreset, climbPreset;
    // index
    public final Button indexShootButton;

    // shooter
    public final Button shootButton;
    public final Button hoodUpButton;
    public final Button hoodDownButton;
    public final Button turretLeftButton;
    public final Button turretRightButton;

    // intake
    public final Button intakeInButton;
    public final Button intakeExtendButton;
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

        driveController = CompoundController.of(CONTROLLER_PORT, PRIMARY, SECONDARY);

        driveController.activate(PRIMARY);

        shootPreset = driveController.getPreset(PRIMARY);

        climbPreset = driveController.getPreset(SECONDARY);

        fixedArmUpManualButton = climbPreset.getDPadButton(Direction.UP);
        fixedArmDownManualButton = climbPreset.getDPadButton(Direction.DOWN);
        dynamicArmUpManualButton = climbPreset.getDPadButton(Direction.LEFT);
        dynamicArmDownManualButton = climbPreset.getDPadButton(Direction.RIGHT);

        fixedArmUpButton = climbPreset.getXButton();
        fixedArmDownButton = climbPreset.getYButton();
        dynamicArmUpButton = climbPreset.getAButton();
        dynamicArmDownButton = climbPreset.getBButton();

        rungClimbButton = climbPreset.getRightBumperButton();

        resetDriveGyroButton = driveController.getRightJoystickButton();

        intakeInButton = shootPreset.getRightBumperButton();
        intakeExtendButton = shootPreset.getXButton();
        intakeSpitButton = shootPreset.getAButton();
        intakeRetractButton = shootPreset.getBButton();

        indexShootButton = shootPreset.getLeftBumperButton();
        shootButton = shootPreset.getLeftTriggerAxis().getButton(0.5);
        hoodUpButton = shootPreset.getDPadButton(Direction.UP);
        hoodDownButton = shootPreset.getDPadButton(Direction.DOWN);
        turretLeftButton = shootPreset.getDPadButton(Direction.LEFT);
        turretRightButton = shootPreset.getDPadButton(Direction.RIGHT);

        driveController.getStartButton().whenPressed(() -> driveController.activate(PRIMARY));

        driveController.getBackButton().whenPressed(() -> driveController.activate(SECONDARY));

        boolean comp = Robot.getInstance().isCompetition();

        if (DRIVE_ENABLED) {
            bindDriveControls();
        }
        if (!comp)
            return;
        if (CLIMB_ENABLED) {
            bindClimbControls();
        }

        if (INDEX_ENABLED)
            bindIndexControls();

        if (INTAKE_ENABLED) {
            bindIntakeControls();
        }
        if (SHOOTER_ENABLED) {
            bindShooterControls();
        }
    }

    // TODO these yay
    public void bindClimbControls() {
        // bruh
    }

    public void bindDriveControls() {
        resetDriveGyroButton.whenPressed(() -> subsystems.drivebaseSubsystem.resetGyroAngle(Rotation2.ZERO));
    }

    public void bindIndexControls() {
        if (SHOOTER_VISION_ENABLED && INDEX_ENABLED) {
            indexShootButton.whileHeld(new IndexShootCommand(subsystems.indexSubsystem));
        }
    }

    public void bindIntakeControls() {
        intakeInButton.whenPressed(new IntakeMotorInCommand(subsystems.intakeSubsystem));
        intakeExtendButton.whenPressed(new IntakeExtendCommand(subsystems.intakeSubsystem));
        intakeSpitButton.whenPressed(new IntakeMotorOutCommand(subsystems.intakeSubsystem));
        intakeRetractButton.whenPressed(new IntakeRetractCommand(subsystems.intakeSubsystem));
    }

    public void bindShooterControls() {
        if (!Subsystems.SubsystemConstants.SHOOTER_TESTING) {
            shootButton.whileHeld(
                    new ShooterTargetCommand(subsystems.shooterSubsystem, subsystems.shooterVisionSubsystem));
            // subsystems.shooterSubsystem.setDefaultCommand(
            // new ShooterTargetCommand(subsystems.shooterSubsystem, subsystems.shooterVisionSubsystem));
            hoodUpButton.whileHeld(new ShooterHoodSetConstantAngleCommand(subsystems.shooterSubsystem,
                    subsystems.shooterSubsystem.getHoodAngle() + 1));
            hoodDownButton.whileHeld(new ShooterHoodSetConstantAngleCommand(subsystems.shooterSubsystem,
                    subsystems.shooterSubsystem.getHoodAngle() - 1));
            turretLeftButton.whileHeld(new ShooterTurretSetAngleCommand(subsystems.shooterSubsystem,
                    subsystems.shooterSubsystem.getTurretAngle() - 5));
            turretRightButton.whileHeld(new ShooterTurretSetAngleCommand(subsystems.shooterSubsystem,
                    subsystems.shooterSubsystem.getTurretAngle() + 5));
        }

    }
}
