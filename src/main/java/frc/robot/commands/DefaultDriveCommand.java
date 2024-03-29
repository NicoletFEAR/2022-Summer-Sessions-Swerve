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
        
        // THIS IS HERMES ONLY DEPLOY THIS WHEN TESTING

        // if there is no driver (or copilot) turning and the robot is driving, cancel out drift
        if (Math.abs(RobotContainer.getXbox0RightX()) < 0.05 && 
            // Math.abs(RobotContainer.getXbox1LeftX()) < 0.05 &&
            (Math.abs(RobotContainer.getXbox0LeftX()) > 0.05 
            || Math.abs(RobotContainer.getXbox0LeftY()) > 0.05)) {
                if (drift < 0) {
                    calculatedRotation = drift;
                } else if (drift > 0) {
                    calculatedRotation = drift;
                } else {
                    calculatedRotation = 0;
                }
        }

        // THIS IS HERMES ONLY DEPLOY THIS WHEN TESTING
        
        SmartDashboard.putNumber("previous angle", previousAngle);
        SmartDashboard.putNumber("drift", drift);
        SmartDashboard.putNumber("calculatedRotation", calculatedRotation);

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(), // + calculatedRotation,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );

        calculatedRotation = 0;
        previousAngle = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees()-180; // get to zero
        SmartDashboard.putNumber("current angle", previousAngle);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
