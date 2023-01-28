// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveDirection;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveDistance2;
import frc.robot.commands.ZeroGyroscope;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  final static DrivetrainSubsystem m_drivebase = new DrivetrainSubsystem();
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
    xbox0.y().onTrue(new AutoDrive(m_drivebase));
    xbox0.x().whileTrue(new DriveDirection(m_drivebase, 0.0, 0.5, 0.0));
    xbox0.b().onTrue(new DriveDistance2(m_drivebase, 1.0, 0.0, 5.0));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      Constants.MAX_VELOCITY_METERS_PER_SECOND,
      Constants.MAX_ACCELERATION_METERS_PER_SEC_SQUARED)
              .setKinematics(m_drivebase.m_kinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
              new Translation2d(1, 0),
              new Translation2d(1, -1)),
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
      trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(Constants.PX_CONTROLLER, 0, 0);
    PIDController yController = new PIDController(Constants.PY_CONTROLLER, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
          Constants.PTHETA_CONTROLLER, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          trajectory,
          m_drivebase::getPose,
          m_drivebase.m_kinematics,
          xController,
          yController,
          thetaController,

          m_drivebase::setModuleStates,
          // constructor wants
          //Consumer<SwerveModuleState[]> outputModuleStates

          m_drivebase);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
          new InstantCommand(() -> m_drivebase.resetOdometry(trajectory.getInitialPose())),
          //m_drivebase.odometer.resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose)
        


          swerveControllerCommand,
          new InstantCommand(() -> m_drivebase.stop()));
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
