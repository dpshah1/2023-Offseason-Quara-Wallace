package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class MoveRollers extends CommandBase {
    public Intake intake;


    public MoveRollers(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

    }



    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (RobotContainer.xController.getRightBumper()) {
            intake.teleopIntakeCommand();
        }
        else if(RobotContainer.xController.getLeftBumper()) {
            intake.teleopOuttakeCommand();
        }
        else {
            intake.stopIntake();
        }

        // System.out.println("Speed: " + speed);
        // System.out.println("Rotation: " + rotation);
        // System.out.println("LeftY: " + RobotContainer.xController.getLeftY());
        // System.out.println("LeftX: " + RobotContainer.xController.getLeftX());
        // System.out.println();


    }

    // Called once the command ends or is interrupted.
    public void end() {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}