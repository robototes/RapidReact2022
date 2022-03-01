package frc.team2412.robot.commands.intake;

import static frc.team2412.robot.subsystem.IndexSubsystem.IndexConstants.INDEX_IN_SPEED;
import static frc.team2412.robot.subsystem.IndexSubsystem.IndexConstants.INDEX_OUT_SPEED;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.INTAKE_IN_SPEED;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.INTAKE_OUT_SPEED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem;
import frc.team2412.robot.subsystem.ShooterSubsystem.ShooterConstants;
import frc.team2412.robot.subsystem.ShooterVisionSubsystem;
import frc.team2412.robot.util.ShooterDataDistancePoint;

public class IntakeBitmapCommand extends CommandBase {

    public final double MISFIRE_VELOCITY = 400;

    // bitmap
    public enum Bitmap {
        // ingesthasball, feederhasball, ingestcorrectcolor, feedercorrectcolor, intakespeed, ingestspeed,
        // feederspeed, misfire
        A(false, false, false, false, INTAKE_IN_SPEED, INDEX_IN_SPEED, INDEX_IN_SPEED, false, "no balls in system"), 
        B(true, false, true, false, INTAKE_IN_SPEED, INDEX_IN_SPEED, INDEX_IN_SPEED, false, "Correct ball in ingest"),
        C(true, false, false, false, INTAKE_IN_SPEED, INDEX_IN_SPEED, INDEX_IN_SPEED, true, "Wrong ball in ingest"), 
        D(false, true, false, true, INTAKE_IN_SPEED, INDEX_IN_SPEED, 0, false, "Correct ball in feeder"), 
        E(false, true, false, false, INTAKE_IN_SPEED, INDEX_IN_SPEED, INDEX_IN_SPEED, true, "Wrong ball in feeder"), 
        F(true, true, true, true, 0, 0, 0, false, "Correct ball in both"),
        G(true, true, false, true, INTAKE_OUT_SPEED, INDEX_OUT_SPEED, 0, false, "Wrong ingest, correct feeder"), 
        H(true, true, true, false, 0, INDEX_IN_SPEED, INDEX_IN_SPEED, true, "Correct ingest, wrong feeder"), 
        I(true, true, false, false, 0, INDEX_IN_SPEED, INDEX_IN_SPEED, true, "Wrong in both");

        boolean ingestSensor, feederSensor, ingestColor, feederColor, shooterMisfire;
        double intakeMotorSpeed, ingestMotorSpeed, feederMotorSpeed;
        String state;

        // enum initializer
        private Bitmap(boolean ingestSensor, boolean feederSensor, boolean ingestColor, boolean feederColor,
                double intakeMotorSpeed, double ingestMotorSpeed, double feederMotorSpeed, boolean shooterMisfire,
                String state) {
            this.ingestSensor = ingestSensor;
            this.feederSensor = feederSensor;
            this.ingestColor = ingestColor;
            this.feederColor = feederColor;
            this.intakeMotorSpeed = intakeMotorSpeed;
            this.ingestMotorSpeed = ingestMotorSpeed;
            this.feederMotorSpeed = feederMotorSpeed;
            this.shooterMisfire = shooterMisfire;
            this.state = state;
        }

        public boolean equals(boolean ingestSensor, boolean feederSensor, boolean ingestColor, boolean feederColor) {
            return this.ingestSensor == ingestSensor &&
                    this.feederSensor == feederSensor &&
                    this.ingestColor == ingestColor &&
                    this.feederColor == feederColor;
        }

        public String toString() {
            return state;
        }
    }

    //
    private IntakeSubsystem intakeSubsystem;
    private IndexSubsystem indexSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private ShooterVisionSubsystem shooterVisionSubsystem;

    String currentState;

    // constructor
    public IntakeBitmapCommand(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem,
            ShooterSubsystem shooterSubsystem, ShooterVisionSubsystem shooterVisionSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexSubsystem = indexSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.shooterVisionSubsystem = shooterVisionSubsystem;
        addRequirements(intakeSubsystem, indexSubsystem, shooterSubsystem, shooterVisionSubsystem);

    }

    @Override
    public void execute() {

        // System.out.println("I AM RUNNING");

        boolean ingestSensor = indexSubsystem.ingestSensorHasBallIn();
        boolean feederSensor = indexSubsystem.feederSensorHasBallIn();
        boolean ingestColor = indexSubsystem.ingestHasCorrectCargo() && ingestSensor;
        boolean feederColor = indexSubsystem.feederHasCorrectCargo() && feederSensor;

        for (Bitmap value : Bitmap.values()) {

            if (value.equals(ingestSensor, feederSensor, ingestColor, feederColor)) {
                indexSubsystem.setBitmapState(value);

                intakeSubsystem.setSpeed(value.intakeMotorSpeed);
                indexSubsystem.setSpeed(value.ingestMotorSpeed, value.feederMotorSpeed);

                double yaw = shooterVisionSubsystem.getDistance() + shooterSubsystem.getTurretAngleBias();
                shooterSubsystem.updateTurretAngle(yaw);

                if (value.shooterMisfire) {
                    shooterSubsystem.setHoodAngle(0);
                    shooterSubsystem.setFlywheelVelocity(MISFIRE_VELOCITY);

                } else {
                    double distance = shooterVisionSubsystem.getDistance() + shooterSubsystem.getDistanceBias();
                    ShooterDataDistancePoint shooterData = ShooterConstants.dataPoints.getInterpolated(distance);
                    shooterSubsystem.setHoodAngle(shooterData.getAngle());
                    shooterSubsystem.setFlywheelRPM(shooterData.getRPM());
                }
            }
        }
    }

    public boolean isFinished() {
        return false;
    }
}
