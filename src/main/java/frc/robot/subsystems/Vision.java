// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    public Vision(NetworkTable table){
        this.table = table;
        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.ta = table.getEntry("ta");

        this.tx.setNumber(Constants.APRIL_TAG_PIPELINE);
        this.ty.setNumber(Constants.APRIL_TAG_PIPELINE);
        this.ta.setNumber(Constants.APRIL_TAG_PIPELINE);

    }


    @Override
    public void periodic() {
        //get values
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}