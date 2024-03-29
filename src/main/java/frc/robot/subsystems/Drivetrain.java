// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  public Drivetrain() {
    RobotContainer.myRobot.arcadeDrive(0, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotContainer.move.execute();

  }

  public void setMovement(double rotSpeed, double moveSpeed) {
    RobotContainer.myRobot.arcadeDrive(rotSpeed, moveSpeed);
  }

  public void stopMovement() {
    RobotContainer.myRobot.arcadeDrive(0, 0);
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
