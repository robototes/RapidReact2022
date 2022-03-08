package frc.team2412.robot.commands.intake;

import static frc.team2412.robot.subsystem.IndexSubsystem.IndexConstants.INDEX_IN_SPEED;
import static frc.team2412.robot.subsystem.IntakeSubsystem.IntakeConstants.INTAKE_IN_SPEED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;
import frc.team2412.robot.subsystem.IntakeSubsystem;

public class IntakeBitmapCommand extends CommandBase {

    public final double MISFIRE_VELOCITY = 400;

    // bitmap
    public enum Bitmap {
        // ingesthasball, feederhasball, ingestcorrectcolor, feedercorrectcolor, intakespeed, ingestspeed,
        // feederspeed, misfire
        // spotless:off
        A(false, false, INTAKE_IN_SPEED, INDEX_IN_SPEED, INDEX_IN_SPEED, "no balls in system"), B(true, false,
                INTAKE_IN_SPEED, INDEX_IN_SPEED, INDEX_IN_SPEED, "Ball in Ingest"), C(false, true, INTAKE_IN_SPEED,
                        INDEX_IN_SPEED, 0, "Ball in Feeder"), D(true, true, 0, 0, 0, "Ball in both");

        // spotless:on

        private boolean ingestSensor, feederSensor;
        private double intakeMotorSpeed, ingestMotorSpeed, feederMotorSpeed;
        private String state;

        // enum initializer
        private Bitmap(boolean ingestSensor, boolean feederSensor, double intakeMotorSpeed, double ingestMotorSpeed,
                double feederMotorSpeed, String state) {
            this.ingestSensor = ingestSensor;
            this.feederSensor = feederSensor;
            this.intakeMotorSpeed = intakeMotorSpeed;
            this.ingestMotorSpeed = ingestMotorSpeed;
            this.feederMotorSpeed = feederMotorSpeed;
            this.state = state;
        }

        public boolean equals(boolean ingestSensor, boolean feederSensor) {
            return this.ingestSensor == ingestSensor &&
                    this.feederSensor == feederSensor;
        }

        public String toString() {
            return state;
        }
    }

    //
    private IntakeSubsystem intakeSubsystem;
    private IndexSubsystem indexSubsystem;

    private Bitmap currentState;

    // constructor
    public IntakeBitmapCommand(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexSubsystem = indexSubsystem;
        addRequirements(intakeSubsystem, indexSubsystem);

    }

    @Override
    public void execute() {

        boolean ingestSensor = indexSubsystem.ingestSensorHasBallIn();
        boolean feederSensor = indexSubsystem.feederSensorHasBallIn();

        for (Bitmap value : Bitmap.values()) {
            if (value.equals(ingestSensor, feederSensor)) {
                currentState = value;
                break;
            }
        }

        indexSubsystem.setBitmapState(currentState);
        intakeSubsystem.setSpeed(currentState.intakeMotorSpeed);
        indexSubsystem.setSpeed(currentState.ingestMotorSpeed, currentState.feederMotorSpeed);
    }

    public boolean isFinished() {
        return false;
    }
}
