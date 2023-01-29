// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveDirection extends CommandBase {
  DrivetrainSubsystem m_drivebase;

  private final double xSpeed, ySpeed, rotation;

  /** Creates a new ExitCommunity. */
  public DriveDirection(DrivetrainSubsystem m_drivebase, double xSpeed, double ySpeed, double rotation) {
    this.m_drivebase = m_drivebase;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotation = rotation;

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
                        m_drivebase.modifySpeed(xSpeed),
                        m_drivebase.modifySpeed(ySpeed),
                        m_drivebase.modifyAngularSpeed(rotation),
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
