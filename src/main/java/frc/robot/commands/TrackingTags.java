// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AprilTag_Auto;
public class TrackingTags extends CommandBase {
  /** Creates a new TrackingTags. */
  AprilTag_Auto m_APTag;
  DrivetrainSubsystem m_driveTrain;

  private DoubleSupplier m_translationXSupplier;
  private DoubleSupplier m_translationYSupplier;
  private DoubleSupplier m_rotationSupplier;

  public TrackingTags(AprilTag_Auto ap, DrivetrainSubsystem dts) {
    m_APTag = RobotContainer.m_AprilTag;
    m_driveTrain = dts;

    m_translationXSupplier = () -> RobotContainer.modifyAxis(0.0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    m_translationYSupplier = () -> RobotContainer.modifyAxis(0.0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    m_rotationSupplier = () -> RobotContainer.modifyAxis(0.0) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(m_APTag, m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(RobotContainer.m_AprilTag.getX());

    m_translationXSupplier = () -> RobotContainer.modifyAxis(0.0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    m_translationYSupplier = () -> RobotContainer.modifyAxis(0.4) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    m_rotationSupplier = () -> RobotContainer.modifyAxis(0.0) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    m_driveTrain.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_driveTrain.getGyroscopeRotation()
                )
    );

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.driveDirection(0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
