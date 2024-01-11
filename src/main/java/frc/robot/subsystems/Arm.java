// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CANSparkMax armMotor;
  public RobotContainer myArm;

    public Arm(CANSparkMax armMotor) {
    this.armMotor = armMotor;
    armMotor.set(0);
    armMotor.getEncoder().setPosition(0.0);
  }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotContainer.moveArmCommand.execute();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
