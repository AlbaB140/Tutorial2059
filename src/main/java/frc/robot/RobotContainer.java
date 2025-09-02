// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final JoystickDriveCommand joystickDriveCommand = new JoystickDriveCommand(drivetrainSubsystem);
  public static Joystick Joystick_1 = new Joystick(0);

  SendableChooser<Command> chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivetrainSubsystem.setDefaultCommand(joystickDriveCommand);

    chooser.addOption("Left Auton.", loadPathplannerTrajectoryToRamseteCommand(
      "C:\\Users\\bundo\\Downloads\\WPILib\\Tutorial2059\\Tutorial2059\\src\\main\\deploy\\deploy\\pathplanner\\paths\\left auton.path", 
      true));
    chooser.addOption("Middle Auton", loadPathplannerTrajectoryToRamseteCommand(
      "C:\\Users\\bundo\\Downloads\\WPILib\\Tutorial2059\\Tutorial2059\\src\\main\\deploy\\deploy\\pathplanner\\paths\\middle auton.path", 
      true));

    Shuffleboard.getTab("Autonomous").add(chooser);
  }

  public Command loadPathplannerTrajectoryToRamseteCommand(String filename, boolean resetOdometry) {
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } 

    catch(IOException exception) {
      DriverStation.reportError("Unable to Open Trajectory" + filename, exception.getStackTrace());
      System.out.println("Unable to Read From File" + filename);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drivetrainSubsystem::getPose,
      new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamseteZeta),
      DrivetrainConstants.kDriveKinematics, null);
      
    if (resetOdometry) {
      return new SequentialCommandGroup(
        new InstantCommand(()->drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    } else {
        return ramseteCommand;
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(drivetrainSubsystem::exampleCondition)
        .onTrue(new JoystickDriveCommand(drivetrainSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(drivetrainSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();
  }
}
