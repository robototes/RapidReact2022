package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.CODRIVER_CONTROLLER_PORT;
import static frc.team2412.robot.Controls.ControlConstants.CONTROLLER_PORT;
import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.DPadButton.Direction;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.team2412.robot.commands.index.IndexShootCommand;
<<<<<<< HEAD
<<<<<<< Updated upstream
import frc.team2412.robot.commands.intake.IntakeInCommand;
=======
import frc.team2412.robot.commands.index.IndexSpitCommand;
import frc.team2412.robot.commands.intake.IntakeExtendCommand;
import frc.team2412.robot.commands.intake.IntakeMotorInCommand;
import frc.team2412.robot.commands.intake.IntakeMotorOutCommand;
>>>>>>> Stashed changes
=======
import frc.team2412.robot.commands.intake.IntakeExtendCommand;
import frc.team2412.robot.commands.intake.IntakeMotorInCommand;
import frc.team2412.robot.commands.intake.IntakeMotorOutCommand;
>>>>>>> 5d44177d0dcc74cb6d1dc9d0e45e315d3faad2af
import frc.team2412.robot.commands.intake.IntakeRetractCommand;
import frc.team2412.robot.commands.shooter.ShooterHoodSetConstantAngleCommand;
import frc.team2412.robot.commands.shooter.ShooterTargetCommand;
import frc.team2412.robot.commands.shooter.ShooterTurretSetAngleCommand;

public class Controls {
    public static class ControlConstants {
        public static final int CONTROLLER_PORT = 0;
        public static final int CODRIVER_CONTROLLER_PORT = 1;
    }

    public XboxController driveController;
    public XboxController codriverController;

<<<<<<< HEAD
<<<<<<< Updated upstream
=======
    // index
    public final Button indexShootButton;
    public final Button indexSpitButton;

>>>>>>> Stashed changes
=======
    // index
    public final Button indexShootButton;

>>>>>>> 5d44177d0dcc74cb6d1dc9d0e45e315d3faad2af
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

        driveController = new XboxController(CONTROLLER_PORT);
        codriverController = new XboxController(CODRIVER_CONTROLLER_PORT);

        fixedArmUpManualButton = codriverController.getDPadButton(Direction.UP);
        fixedArmDownManualButton = codriverController.getDPadButton(Direction.DOWN);
        dynamicArmUpManualButton = codriverController.getDPadButton(Direction.LEFT);
        dynamicArmDownManualButton = codriverController.getDPadButton(Direction.RIGHT);

        fixedArmUpButton = codriverController.getXButton();
        fixedArmDownButton = codriverController.getYButton();
        dynamicArmUpButton = codriverController.getAButton();
        dynamicArmDownButton = codriverController.getBButton();

        rungClimbButton = codriverController.getRightBumperButton();

        resetDriveGyroButton = driveController.getRightJoystickButton();

        intakeInButton = driveController.getRightBumperButton();
        intakeExtendButton = driveController.getXButton();
        intakeSpitButton = driveController.getAButton();
        intakeRetractButton = driveController.getBButton();

        indexShootButton = driveController.getLeftTriggerAxis().getButton(0.5);
        indexSpitButton = driveController.getLeftBumperButton();
        codriverIndexShootButton = codriverController.getXButton();
        codriverIndexSpitButton = codriverController.getAButton();

        shootButton = driveController.getRightTriggerAxis().getButton(0.5);

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

    // TODO these yay
    public void bindClimbControls() {

    }

    public void bindDriveControls() {
        resetDriveGyroButton.whenPressed(() -> {
            subsystems.drivebaseSubsystem.resetGyroAngle(Rotation2.ZERO);
        });
    }

    public void bindIndexControls() {
<<<<<<< Updated upstream
        if (SHOOTER_ENABLED && SHOOTER_VISION_ENABLED && INDEX_ENABLED) {
<<<<<<< HEAD
            shootButton.whenPressed(new IndexShootCommand(subsystems.indexSubsystem));
=======
        if (INDEX_ENABLED) {
            indexShootButton.whileHeld(new IndexShootCommand(subsystems.indexSubsystem));
            indexSpitButton.whileHeld(new IndexSpitCommand(subsystems.indexSubsystem));
>>>>>>> Stashed changes
=======
            indexShootButton.whileHeld(new IndexShootCommand(subsystems.indexSubsystem));
>>>>>>> 5d44177d0dcc74cb6d1dc9d0e45e315d3faad2af
        }
    }

    public void bindIntakeControls() {
<<<<<<< HEAD
<<<<<<< Updated upstream
        intakeInButton.whenPressed(new IntakeInCommand(subsystems.indexSubsystem, subsystems.intakeSubsystem));
        intakeSpitButton.whenPressed(new SpitBallCommand(subsystems.indexSubsystem, subsystems.intakeSubsystem));
=======
        intakeInButton.whileHeld(new IntakeMotorInCommand(subsystems.intakeSubsystem));
        intakeExtendButton.whenPressed(new IntakeExtendCommand(subsystems.intakeSubsystem));
        intakeSpitButton.whileHeld(new IntakeMotorOutCommand(subsystems.intakeSubsystem));
>>>>>>> 5d44177d0dcc74cb6d1dc9d0e45e315d3faad2af
        intakeRetractButton.whenPressed(new IntakeRetractCommand(subsystems.intakeSubsystem));
=======
        if (INTAKE_ENABLED) {
            intakeInButton.whileHeld(new IntakeMotorInCommand(subsystems.intakeSubsystem));
            intakeExtendButton.whenPressed(new IntakeExtendCommand(subsystems.intakeSubsystem));
            intakeSpitButton.whileHeld(new IntakeMotorOutCommand(subsystems.intakeSubsystem));
            intakeRetractButton.whenPressed(new IntakeRetractCommand(subsystems.intakeSubsystem));
        }
>>>>>>> Stashed changes
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
