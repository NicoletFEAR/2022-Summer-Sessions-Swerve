// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DrivetrainSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class DriveDistance extends CommandBase {
//   private DrivetrainSubsystem m_drivebase;
//   private double xDistance, yDistance, time;
  
//   public DriveDistance(DrivetrainSubsystem m_drivebase, double xDistance, double yDistance, double time) {
//     this.xDistance = xDistance;
//     this.yDistance = yDistance;
//     this.time = time;
//     this.m_drivebase = m_drivebase;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_drivebase);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_drivebase.driveDistance(xDistance, yDistance, time);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_drivebase.driveDirection(0, 0, 0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }

// }
