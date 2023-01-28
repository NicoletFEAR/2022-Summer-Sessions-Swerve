// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;


public class AprilTag_Auto extends SubsystemBase {
  /** Creates a new AprilTag_Auto. */
  public static AprilTag_Auto camera;
  
  private NetworkTable m_visionTable;
  private NetworkTableEntry m_visV;
  private NetworkTableEntry m_visX;
  private NetworkTableEntry m_visY;
  private NetworkTableEntry m_visA;

  // Change this to match the name of your camera

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.01;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  public double angle;
  public double area;
  public int fidId;

  public AprilTag_Auto(String name) {
    AprilTag_Auto.camera = this;
    m_visionTable = NetworkTableInstance.getDefault().getTable(name);
    m_visV = m_visionTable.getEntry("tv");
    m_visX = m_visionTable.getEntry("tx");
    m_visY = m_visionTable.getEntry("ty");
    m_visA = m_visionTable.getEntry("ta");
  }


  public double angle(){
    return angle;
  }

  
  public double getX() {
    return m_visX.getDouble(0.0);
  }

  public double getY() {
    return m_visY.getDouble(0.0);
  }

  public double getA() {
    return m_visA.getDouble(0.0);
  }



  public double getV() {
    return m_visV.getDouble(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    //forwardSpeed = -RobotContainer.getRightY();

        // Vision-alignment mode
        // Query the latest result from PhotonVision
        //var result = camera.getLatestResult();

       
            // Calculate angular turn power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            //System.out.println("target found");
            //angle = result.getBestTarget().getYaw();
            //area = result.getBestTarget().getArea();
            //fidId = result.getBestTarget().getFiducialId();

            SmartDashboard.putNumber("aprilTag X", getX());

  }
}