// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class MoveWrist extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Wrist m_subsystem;
  private PIDController pid = new PIDController(Constants.P_WRIST, Constants.I_WRIST, Constants.D_WRIST);
  private double wristSetpoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveWrist(Wrist subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    wristSetpoint = m_subsystem.wristMotor.getEncoder().getPosition();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Wrist Setpoint", wristSetpoint);
    
    // Move wrist up
    if(RobotContainer.xController.getRightX() > 0.5)
    {
        m_subsystem.wristMotor.set(Constants.WRIST_SPEED);
        pid.reset();
        wristSetpoint = m_subsystem.wristMotor.getEncoder().getPosition();
    }
    // Move wrist down
    else if(RobotContainer.xController.getRightX() < -0.5)
    {
        m_subsystem.wristMotor.set(-Constants.WRIST_SPEED);
        pid.reset();
        wristSetpoint = m_subsystem.wristMotor.getEncoder().getPosition();
    }
    // Use PID to keep the wrist in place
    else
    {
        double currentPosition = m_subsystem.myWrist.wristMotor.getEncoder().getPosition();
        m_subsystem.wristMotor.set(pid.calculate(currentPosition, wristSetpoint));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
