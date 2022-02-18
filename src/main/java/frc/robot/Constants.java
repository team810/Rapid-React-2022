// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    // JOYSTICK CONFIG
    public static final int GP = 2;
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;

    public static final int XAXIS = 0;
    public static final int YAXIS = 1;

    // JOYSTICK BUTTONS
    public static final int TRIGGER_BUTTON = 1;
    public static final int MIDDLE_BUTTON = 2;
    public static final int LEFT_BUTTON = 3;
    public static final int RIGHT_BUTTON = 4;
    public static final int SIDE_BUTTON1 = 5;
    public static final int SIDE_BUTTON2 = 6;
    public static final int SIDE_BUTTON3 = 7;

    // GAMEPAD BUTTONS
    public static final int X = 1;
    public static final int A = 2;
    public static final int B = 3;
    public static final int Y = 4;
    public static final int LT = 7;
    public static final int RT = 8;

    // DRIVETRAIN (CAN)
    public final static int FRONTL = 4;
    public final static int FRONTR = 5;
    public final static int BACKL = 3;
    public final static int BACKR = 6;

    // INTAKE (SPARK)
    public static final int INTAKE_MOTOR = 1;
    public static final int INTAKE_SOLENOID_1 = 14;
    public static final int INTAKE_SOLENOID_2 = 15;

    // SHOOTER (CAN)
    public static final int SHOOTER_TOP = 2;
    public static final int SHOOTER_BOTTOM = 1;

    // CLIMBER (CAN)
    public static final int CLIMBER_MOTOR = 7;

    public final static int HOOKL_1 = 1;
    public final static int HOOKL_2 = 2;
    public final static int HOOKR_1 = 3;
    public final static int HOOKR_2 = 4; 
    public final static double CLIMBER_REVS = 15.7079633;

    // FEEDER (SPARK)
    public static final int FEEDER_MOTOR = 0;

    //SHOOTER PID VALUES
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