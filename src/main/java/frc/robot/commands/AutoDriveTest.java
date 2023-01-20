package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class AutoDriveTest extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_drivebase;
  private Double x_speed;
  private Double y_speed;

  public AutoDriveTest(DrivetrainSubsystem subsystem, Double xSpeed, Double ySpeed) {
    m_drivebase = subsystem;
    x_speed = xSpeed;
    y_speed = ySpeed;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.driveDirection(x_speed, y_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}