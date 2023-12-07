// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Imports subsystems and commands 

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Move;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.ExampleCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Drivetrain motors 
  public static WPI_VictorSPX rightLeader = new WPI_VictorSPX(Constants.RightLeader);
  public static WPI_VictorSPX rightFollower = new WPI_VictorSPX(Constants.RightFollower);
  public static WPI_VictorSPX leftLeader = new WPI_VictorSPX(Constants.LeftLeader);
  public static WPI_VictorSPX leftFollower = new WPI_VictorSPX(Constants.LeftFollower);

  // Intake motors
  public static CANSparkMax leftIntakeMotor = new CANSparkMax(Constants.LeftIntakeMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static CANSparkMax rightIntakeMotor = new CANSparkMax(Constants.RightIntakeMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

  // Motor groups
  public static MotorControllerGroup leftDrive = new MotorControllerGroup(leftLeader, leftFollower);
  public static MotorControllerGroup rightDrive = new MotorControllerGroup(rightLeader, rightFollower);

  public static DifferentialDrive myRobot = new DifferentialDrive(leftDrive, rightDrive);
  public static Drivetrain drivetrain = new Drivetrain();
  public static Move move = new Move(drivetrain);

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public static XboxController xController = new XboxController(Constants.XBOX_PORT);
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final Intake myIntake = new Intake(leftIntakeMotor, rightIntakeMotor);

  public static CANSparkMax wristMotor = new CANSparkMax(Constants.WRIST_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static Wrist myWrist = new Wrist(wristMotor);

  public static MoveWrist moveWristCommand = new MoveWrist(myWrist);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(xController, Constants.TELEOP_INTAKE_BUTTON).whileTrue(myIntake.teleopIntakeCommand());
    new JoystickButton(xController, Constants.TELEOP_OUTTAKE_BUTTON).whileTrue(myIntake.teleopOuttakeCommand());
    new JoystickButton(xController, Constants.STOP_INTAKE_BUTTON).whileTrue(myIntake.stopIntake());
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
