// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CANSparkMax wristMotor;
  public RobotContainer myWrist;

    public Wrist(CANSparkMax wristMotor) {
    this.wristMotor = wristMotor;
    wristMotor.set(0);
    wristMotor.getEncoder().setPosition(0.0);
  }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotContainer.moveWristCommand.execute();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
