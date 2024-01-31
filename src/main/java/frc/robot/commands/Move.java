package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class Move extends CommandBase {
  public Drivetrain drivetrain;


  public Move(Drivetrain mDrivetrain) {
    drivetrain = mDrivetrain;
    addRequirements(mDrivetrain);

  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Calls a function to move the robot depending on the driveMode constant 
    double speed = RobotContainer.xController.getLeftY();
    double rotation = RobotContainer.xController.getLeftX();
    drivetrain.setMovement(rotation * Constants.ROTATION_SPEED, speed * Constants.DRIVE_SPEED);

    // System.out.println("Speed: " + speed);
    // System.out.println("Rotation: " + rotation);
    // System.out.println("LeftY: " + RobotContainer.xController.getLeftY());
    // System.out.println("LeftX: " + RobotContainer.xController.getLeftX());
    // System.out.println();


  }

  // Called once the command ends or is interrupted.
  public void end() {
    RobotContainer.myRobot.tankDrive(0, 0);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}