// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  /// create a spark
  
  CANSparkMax leftIntakeMotor;
  CANSparkMax rightIntakeMotor;
  
    
  public Intake(CANSparkMax leftIntakeMotor, CANSparkMax rightIntakeMotor) {
    this.leftIntakeMotor = leftIntakeMotor;
    this.rightIntakeMotor = rightIntakeMotor;
    leftIntakeMotor.follow(rightIntakeMotor, true);
    rightIntakeMotor.set(0);
  }
    
   @Override
   public void periodic() {
     RobotContainer.intakeCommand.execute();
   }

  public void teleopIntakeCommand(){
    System.out.println("ran intake");
      rightIntakeMotor.set(Constants.INTAKE_SPEED);
  } 

  public void teleopOuttakeCommand(){
    System.out.println("ran outake");
      rightIntakeMotor.set(Constants.OUTTAKE_SPEED);
  }

  public void stopIntake(){
      rightIntakeMotor.set(0);
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}