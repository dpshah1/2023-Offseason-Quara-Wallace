// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/**
 * Command to tilt the robot towards a target.
 */
public class TiltTowardsTarget extends Command {
    private final Vision visionSubsystem;
    private final Drivetrain drivetrainSubsystem;

    // Constants
	private final double TILT_TOLERANCE = 0.5;
    private final double kP = 0.02; // might need adjusting
	private final double MIN_SPEED = 0.2;

    private PIDController pid;

    /**
     * Creates a new TiltTowardsTarget command.
     *
     * @param visionSubsystem The vision subsystem used to calculate the target offset.
     * @param drivetrainSubsystem The drivetrain subsystem used to control the robot's movement.
     */
    public TiltTowardsTarget(Vision visionSubsystem, Drivetrain drivetrainSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;

        // Declare subsystem dependencies.
        addRequirements(drivetrainSubsystem);

        // Initialize the PID controller.
        pid = new PIDController(kP, 0, 0);
        pid.setTolerance(TILT_TOLERANCE);
        pid.setSetpoint(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double offset = visionSubsystem.calcOffset();


		double rotSpeed = Math.max(Math.abs(pid.calculate(offset)), MIN_SPEED);
		System.out.println("Calculated rot Speed = " + rotSpeed);

		System.out.println("Calculated position error = " + pid.getPositionError());
		
		// turn right
		if(offset > 0)
		{
			drivetrainSubsystem.setMovement(rotSpeed, 0);
		}
		// turn left
		else if(offset < 0)
		{
			drivetrainSubsystem.setMovement(-rotSpeed, 0);
		}
		
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopMovement();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(visionSubsystem.calcOffset()) < TILT_TOLERANCE;
    }
}