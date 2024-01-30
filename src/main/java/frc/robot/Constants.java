// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Drivetrain ports
    public static final int RightLeader = 1;
    public static final int RightFollower = 4;
    public static final int LeftLeader = 3;
    public static final int LeftFollower = 2;

    
    // Intake Ports
    public static final int LeftIntakeMotorPort = 12; // Change these to the correct ports
    public static final int RightIntakeMotorPort = 11; // Change these to the correct ports

    // Xbox controller buttons
    public static final int XBOX_PORT = 0;
    public static final int TELEOP_OUTTAKE_BUTTON = 5;
    public static final int TELEOP_INTAKE_BUTTON = 6;
    public static final int STOP_INTAKE_BUTTON = 3;

    public static final int TILT_BUTTON = 1;

    // Drivetrain speeds
    public static final double DRIVE_SPEED = 0.5;
    public static final double ROTATION_SPEED = 0.5;
    
    // Intake speeds
    public static final double INTAKE_SPEED = 0.2;
    public static final double OUTTAKE_SPEED = -0.8;

    // Wrist and arm stuff
    public static final int WRIST_MOTOR_PORT = 10; // find out actual id
    public static final int ARM_MOTOR_PORT = 13;
    public static final double WRIST_SPEED = 0.2;
    public static final double ARM_SPEED = 0.3;
    public static final double P_WRIST = 0.1;
    public static final double I_WRIST = 0;
    public static final double D_WRIST = 0;
    public static final double P_ARM = 0.1;
    public static final double I_ARM = 0;
    public static final double D_ARM = 0;

    // Vision stuff
    public static final int APRIL_TAG_PIPELINE = 4;

}
