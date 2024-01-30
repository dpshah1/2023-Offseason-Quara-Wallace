// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;


/** An example command that uses an example subsystem. */
public class TiltTowardsTarget extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Vision visionSubsystem;
  private final Drivetrain drivetrainSubsystem;

  private final double TILT_TOLERANCE = 0.5;
  private final double kP = 0.02; // migt need adjusting

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TiltTowardsTarget(Vision visionSubsystem, Drivetrain drivetrainSubsytem) {
    this.visionSubsystem = visionSubsystem;
    this.drivetrainSubsystem = drivetrainSubsytem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      System.out.println("initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      System.out.println("execute");
    // A target has been found
    double offset = visionSubsystem.calcOffset();
    if (offset != 0.0) {
        // Robot neeeds to spin clockwise
        double speed = Math.max(kP * Math.abs(offset), 0.2);
        System.out.println("Speed: " + speed);
        if (offset > 0 && Math.abs(offset) > TILT_TOLERANCE) {
            System.out.println("Clockwise movement");
            drivetrainSubsystem.setMovement(speed, 0);
        }
        // Robot needs to spin counter-clockwise
        else if (offset < 0 && Math.abs(offset) > TILT_TOLERANCE) {
            System.out.println("Counterclockwise movement");
            drivetrainSubsystem.setMovement(-speed, 0);
        }



    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("end");
      drivetrainSubsystem.stopMovement();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      System.out.println("is finished");
    if (Math.abs(visionSubsystem.calcOffset()) <= kP) {
        return true;
    }
    else {
        return false;
    }
  }
}