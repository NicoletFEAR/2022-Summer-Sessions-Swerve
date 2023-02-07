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

  double xSpeed, ySpeed;

  public TrackingTags(AprilTag_Auto ap, DrivetrainSubsystem dts) {
    m_APTag = RobotContainer.m_AprilTag;
    m_driveTrain = dts;
  // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(m_APTag, m_driveTrain);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(m_APTag.getX())>1.0){
     xSpeed = m_APTag.getX()/24;
    }
    if(m_APTag.getA()>1.05&&m_APTag.getA()!=0){
      ySpeed = (m_APTag.getA()-1)/3;
    }
    else if(m_APTag.getA()<0.95&&m_APTag.getA()!=0){
      ySpeed = -(1-(m_APTag.getA()/3));
    }
    else{
      ySpeed = 0.0;
    }
    

    System.out.println(m_APTag.getA());
    m_driveTrain.driveDirection(ySpeed, xSpeed, 0);


    
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
