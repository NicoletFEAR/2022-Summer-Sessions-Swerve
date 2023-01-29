// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.DrivetrainSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class DriveDistance2 extends SequentialCommandGroup {
//   private DrivetrainSubsystem m_drivebase;
//   private double xDistance, yDistance, time;

//   /** Creates a new DriveDistance2. */
//   public DriveDistance2(DrivetrainSubsystem m_drivebase, double xDistance, double yDistance, double time) {
//     this.xDistance = xDistance;
//     this.yDistance = yDistance;
//     this.time = time;
//     this.m_drivebase = m_drivebase;

//     System.out.println("executing");

//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       new DriveDistance(m_drivebase, xDistance, yDistance, time).withTimeout(time)
//     );
//   }
// }
