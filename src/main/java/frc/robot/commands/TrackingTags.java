// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.AprilTag_Auto;
public class TrackingTags extends CommandBase {
  /** Creates a new TrackingTags. */
  AprilTag_Auto m_APTag;
  DrivetrainSubsystem m_DriveTrain;

  public TrackingTags(AprilTag_Auto ap, DrivetrainSubsystem dts) {
    m_APTag = ap;
    m_DriveTrain = dts;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_APTag, m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_APTag.getX()>Constants.APRILTAG_CENTERING_DEADZONE){
      m_DriveTrain.driveDirection(0, 0.3, 0);
    }
    else if(m_APTag.getX()<-Constants.APRILTAG_CENTERING_DEADZONE){
      m_DriveTrain.driveDirection(0, -0.3, 0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //new DriveDirection(m_DriveTrain, 0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
