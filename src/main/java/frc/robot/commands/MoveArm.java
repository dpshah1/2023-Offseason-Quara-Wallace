// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class MoveArm extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Arm m_subsystem;
  private PIDController pid = new PIDController(Constants.P_ARM, Constants.I_ARM, Constants.D_ARM);
  private double armSetpoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveArm(Arm subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    armSetpoint = m_subsystem.armMotor.getEncoder().getPosition();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Arm Setpoint", armSetpoint);
    
    // Move wrist up
    if(RobotContainer.xController.getRightY() > 0.5)
    {
        m_subsystem.armMotor.set(Constants.ARM_SPEED);
        pid.reset();
        armSetpoint = m_subsystem.armMotor.getEncoder().getPosition();
    }
    // Move wrist down
    else if(RobotContainer.xController.getRightY() < -0.5)
    {
        m_subsystem.armMotor.set(-Constants.ARM_SPEED);
        pid.reset();
        armSetpoint = m_subsystem.armMotor.getEncoder().getPosition();
    }
    // Use PID to keep the wrist in place
    else
    {
        double currentPosition = m_subsystem.myArm.armMotor.getEncoder().getPosition();
        m_subsystem.armMotor.set(pid.calculate(currentPosition, armSetpoint));
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
