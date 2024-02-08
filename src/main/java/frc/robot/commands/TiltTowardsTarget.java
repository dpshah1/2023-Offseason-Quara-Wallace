package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.*;

import java.util.Set;

public class TiltTowardsTarget extends Command {
    private final Vision visionSubsystem;
    private final Drivetrain drivetrainSubsystem;

    private final double TILT_TOLERANCE = .01;
    private final double kP = 0.03;
    private final double MIN_SPEED = 0.2;

    private PIDController pid;

    public TiltTowardsTarget(Vision visionSubsystem, Drivetrain drivetrainSubsystem) {
        if (visionSubsystem == null || drivetrainSubsystem == null) {
            throw new IllegalArgumentException("Subsystems cannot be null");
        }

        this.visionSubsystem = visionSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);

        pid =
                new PIDController(kP, 0, 0);
        pid.setTolerance(TILT_TOLERANCE);
        pid.setSetpoint(0);
    }

    @Override
    public void execute() {


        double offset;
        try {
            offset = visionSubsystem.calcOffset();
        } catch (Exception e) {
            System.err.println("Error calculating offset: " + e.getMessage());
            offset = 0;
        }

        if (Math.abs(offset) <= TILT_TOLERANCE) {
            drivetrainSubsystem.stopMovement();
            return;
        }

        double rotSpeed = Math.max(Math.abs(pid.calculate(offset)), MIN_SPEED);
        System.out.println("speed" + rotSpeed);
        if(offset > 0) {
            drivetrainSubsystem.setMovement(rotSpeed, 0);
        } else if(offset < 0) {
            drivetrainSubsystem.setMovement(-rotSpeed, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        double offset;
        try {
            offset = visionSubsystem.calcOffset();
        } catch (Exception e) {
            System.err.println("Error calculating offset: " + e.getMessage());
            return true;
        }
        return Math.abs(offset) <= TILT_TOLERANCE;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrainSubsystem);
    }
}