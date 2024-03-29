// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.47625; // FIXME Measure and set trackwidth
    
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.47625; // FIXME Measure and set wheelbase

    /**
     * The diameter of a wheel in feet
     */
    public static final double DRIVETRAIN_WHEEL_DIAMETER = 4.0/12.0;

    /**
     * The circumference of a wheel in feet
     */
    public static final double DRIVETRAIN_WHEEL_CIRCUMFERENCE = Math.PI * DRIVETRAIN_WHEEL_DIAMETER;

    // public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 12; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(210.1+90); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 18;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 17; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 7;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(309.4+90); 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 14; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 13; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(215.8+90);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 5; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(213.2+90);
}
