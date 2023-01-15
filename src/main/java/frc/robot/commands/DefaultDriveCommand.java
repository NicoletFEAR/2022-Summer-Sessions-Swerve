package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private double drift;
    private double previousAngle;
    private double calculatedRotation;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier      translationXSupplier,
                               DoubleSupplier      translationYSupplier,
                               DoubleSupplier      rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        drift = (m_drivetrainSubsystem.getGyroscopeRotation().getDegrees()-180 - previousAngle);
        if (Math.abs(RobotContainer.getXbox0LeftX()) > 0.03 && Math.abs(RobotContainer.getXbox0LeftY()) > 0.03 && Math.abs(RobotContainer.getXbox0RightX()) < 0.03 ) {
            if (drift <= 180) {
                calculatedRotation = drift;
            } else {
                calculatedRotation = -drift;
            }
        }

        SmartDashboard.putNumber("drift", drift);
        SmartDashboard.putNumber("calculatedRotation", calculatedRotation);

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble() + calculatedRotation,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );

        previousAngle = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees()-180; // get to zero
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
