// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AprilTag_Auto;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrackingTags2 extends CommandBase {
  DrivetrainSubsystem m_drivebase;
  AprilTag_Auto m_APTag;

  private DoubleSupplier m_translationXSupplier;
  private DoubleSupplier m_translationYSupplier;
  private DoubleSupplier m_rotationSupplier;
  private double x;
  /** Creates a new ExitCommunity. */
  public TrackingTags2(DrivetrainSubsystem m_drivebase, AprilTag_Auto ap) {
    this.m_drivebase = m_drivebase;
    m_APTag = ap;
    x = ap.getX();
    if(x>2){
      m_translationXSupplier = () -> RobotContainer.modifyAxis(0.0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
      m_translationYSupplier = () -> RobotContainer.modifyAxis(0.4) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
      m_rotationSupplier = () -> RobotContainer.modifyAxis(0.0) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  
    } else if(x<-2){
      m_translationXSupplier = () -> RobotContainer.modifyAxis(0.0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
      m_translationYSupplier = () -> RobotContainer.modifyAxis(-0.4) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
      m_rotationSupplier = () -> RobotContainer.modifyAxis(0.0) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    } else{
      m_translationXSupplier = () -> RobotContainer.modifyAxis(0.0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
      m_translationYSupplier = () -> RobotContainer.modifyAxis(0.0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
      m_rotationSupplier = () -> RobotContainer.modifyAxis(0.0) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_drivebase.zeroGyroscope();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivebase.getGyroscopeRotation()
                )
    );
    // m_drivebase.driveDirection(.3, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.driveDirection(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
