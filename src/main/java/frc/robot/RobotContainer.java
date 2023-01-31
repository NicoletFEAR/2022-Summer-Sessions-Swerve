// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveDirection;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveDistance2;
import frc.robot.commands.TrackingTags;
import frc.robot.commands.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.AprilTag_Auto;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  final static DrivetrainSubsystem m_drivebase = new DrivetrainSubsystem();
  public final static AprilTag_Auto m_AprilTag = new AprilTag_Auto("limelight");
  static final CommandXboxController xbox0 = new CommandXboxController(0);
  Trigger backButton = xbox0.back();
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    //  Left stick Y axis -> forward and backwards movement
    //  Left stick X axis -> left and right movement
    //  Right stick X axis -> rotation
    m_drivebase.setDefaultCommand(new DefaultDriveCommand(
            m_drivebase,
            () -> modifyAxis(xbox0.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(xbox0.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(xbox0.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

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
    // Back button zeros the gyroscope
    backButton.onTrue(new ZeroGyroscope(m_drivebase));
    //xbox0.y().onTrue(new AutoDrive(m_drivebase));
    //xbox0.x().whileTrue(new DriveDirection(m_drivebase, 0.0, 0.5, 0.0));
    //xbox0.b().onTrue(new DriveDistance2(m_drivebase, 1.0, 0.0, 5.0));
    //xbox0.a().whileTrue(new TrackingTags(m_AprilTag, m_drivebase));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  public static double getXbox0LeftY() {
    return xbox0.getLeftY();
  }

  public static double getXbox0LeftX() {
    return xbox0.getLeftX();
  }

  public static double getXbox0RightX() {
    return xbox0.getRightX();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.03);

    // Square the axis but keeps the negative if it's negative
    value = Math.copySign(value * value, value);

    return value;
  }
}
