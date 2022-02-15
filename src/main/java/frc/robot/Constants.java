// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int LEFT_JOYSTICK = 1;
    public static final int RIGHT_JOYSTICK = 2;

    public static final int XAXIS = 0;
    public static final int YAXIS = 1;

    public static final int TRIGGER_BUTTON = 1;
    public static final int MIDDLE_BUTTON = 2;
    public static final int LEFT_BUTTON = 3;
    public static final int RIGHT_BUTTON = 4;

    public final static int FRONTL = 0;
    public final static int FRONTR = 1;
    public final static int BACKL = 2;
    public final static int BACKR = 3;

    public static final int INTAKE_MOTOR = 4;
    public static final int INTAKE_SOLENOID = 0;

    public static final int SHOOTER_TOP = 5;
    public static final int SHOOTER_BOTTOM = 6;

    public static final int FEEDER_MOTOR = 7;

    public static final int CLIMBER_MOTOR = 8;
    public final static int HOOKL = 1;
    public final static int HOOKR = 2; 
    public final static double CLIMBER_REVS = 6.28;

    //CAN Feeder
    public final static int FEEDER = 4;


    //LIMELIGHT
    public static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


    public static final NetworkTableEntry tx = table.getEntry("tx");
    public static final NetworkTableEntry ty = table.getEntry("ty");
    public static final NetworkTableEntry ta = table.getEntry("ta");
    public static final NetworkTableEntry tv = table.getEntry("tv");
    public static final NetworkTableEntry ledMode = table.getEntry("ledMode");
    public static final NetworkTableEntry camMode = table.getEntry("camMode");
    public static final NetworkTableEntry pipeline = table.getEntry("pipeline");
    public static final NetworkTableEntry stream = table.getEntry("stream");

    
    //SHOOTER PID VALUE
    //top 
    public static final double kPTop = 0;
    public static final double kITop = 0;
    public static final double kDTop = 0;
    public static final double kFTop = 0; 

    //bottom
    public static final double kPBottom = 0;
    public static final double kIBottom = 0;
    public static final double kDBottom = 0;
    public static final double kFBottom = 0; 
}