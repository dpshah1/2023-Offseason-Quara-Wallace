package frc.robot.commands;

import java.util.Map;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveMode;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RangeSetter;
import frc.robot.RobotContainer;

public class Move extends CommandBase {
  private double spdMult;
  private double rotMult;
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
    if (driveMode == DriveMode.arcadeDrive) {
      double speed = RobotContainer.xController.getLeftY();
      double rotation = RobotContainer.xController.getLeftX();
      RobotContainer.myRobot.arcadeDrive(rotation * rotMult, -spdMult * speed);
      //System.out.println("leftX: " + RobotContainer.xController.getLeftX() + " | leftY: " + RobotContainer.xController.getLeftY() + " | rightX: " + RobotContainer.xController.getRightX() + " | " + RobotContainer.xController.getRightY());
    }
    else if (driveMode == DriveMode.tankDrive) {
      // Needs to be negative or else it goes backwards... 
      double left = RobotContainer.xController.getLeftY();
      double right = -RobotContainer.xController.getRightY();
      RobotContainer.myRobot.tankDrive(spdMult * left,spdMult * right);
    }
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