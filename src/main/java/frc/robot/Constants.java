// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants{

    //USB

    public final static int LEFT_JOYSTICK = 0;
    public final static int RIGHT_JOYSTICK = 1;
    

    //CAN - DRIVETRAIN
    public final static int FRONTL = 8;
    public final static int FRONTR = 1;
    public final static int BACKL = 2;
    public final static int BACKR = 3;

    //CAN - SHOOTER
    public final static int SHOOTER = 7;
    public final static int ACTIVE = 5;
    public final static int HOOD = 6;


    //CAN - INTAKE
    public final static int INTAKE = 4;

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




}
