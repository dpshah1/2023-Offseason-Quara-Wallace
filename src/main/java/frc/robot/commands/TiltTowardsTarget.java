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

    private final double TILT_TOLERANCE = 0.5;
    private final double kP = 0.03; // might need adjusting

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
        pid = new PIDController(kP, 0, 0.3);
        pid.setTolerance(TILT_TOLERANCE);
        pid.setSetpoint(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double offset = visionSubsystem.calcOffset();


        if(offset != 0) {
            double rotSpeed = pid.calculate(offset) * -1;
            System.out.println(rotSpeed);
            drivetrainSubsystem.setMovement(rotSpeed, 0);
        } else {
            drivetrainSubsystem.stopMovement();
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
        return pid.atSetpoint();
    }
}