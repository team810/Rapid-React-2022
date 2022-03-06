// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
=======
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
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
<<<<<<< HEAD
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
    public static final int B = 3;
    public static final int A = 4;
    public static final int LT = 7;
    public static final int RT = 8;

    // DRIVETRAIN (CAN)
=======
public final class Constants{

    //USB
    public final static int LEFT_JOYSTICK = 0;
    public final static int RIGHT_JOYSTICK = 1;

    public final static int GAMEPAD = 2;
    

    //CAN - DRIVETRAIN
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
    public final static int FRONTL = 4;
    public final static int FRONTR = 5;
    public final static int BACKL = 3;
    public final static int BACKR = 6;

<<<<<<< HEAD
    // INTAKE (SPARK)
    public static final int INTAKE_MOTOR = 1;
    public static final int INTAKE_SOLENOID_1 = 14;
    public static final int INTAKE_SOLENOID_2 = 15;

    // SHOOTER (CAN)
    public static final int SHOOTER_TOP = 2;
    public static final int SHOOTER_BOTTOM = 1;

    // CLIMBER (CAN)
    public static final int CLIMBER_MOTOR = 7;
=======
    //CAN - SHOOTER
    public final static int SHOOTER_TOP = 2;
    public final static int SHOOTER_BOTTOM = 1;

    //INTAKE
    public final static int INTAKE = 1;

    //FEEDER
    public final static int FEEDER = 0;

    //CAN CLIMBER
    public final static int CLIMB = 7;
    
    
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


    public static final int XAXIS = 0;
    public static final int YAXIS = 1;
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f

    public final static int HOOKL_1 = 1;
    public final static int HOOKL_2 = 2;
    public final static int HOOKR_1 = 3;
    public final static int HOOKR_2 = 4; 
    public final static double CLIMBER_REVS = 15.7079633;

    // FEEDER (SPARK)
    public static final int FEEDER_MOTOR = 0;

    //SHOOTER PID VALUES
    //top 
<<<<<<< HEAD
    public static final double kPTop = 0;
=======
    public static final double kPTop = 0.0002;
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
    public static final double kITop = 0;
    public static final double kDTop = 0;
    public static final double kFTop = 0; 

    //bottom
<<<<<<< HEAD
    public static final double kPBottom = 0;
    public static final double kIBottom = 0;
    public static final double kDBottom = 0;
    public static final double kFBottom = 0; 
}
=======
    public static final double kPBottom = 0.0002;
    public static final double kIBottom = 0;
    public static final double kDBottom = 0;
    public static final double kFBottom = 0; 

    //AUTONOMOUS
    public static final double ksVolts = 0.33251;
    public static final double kvVoltSecondsPerMeter = 2.5645;
    public static final double kaVoltSecondsSquaredPerMeter = 0.86909;

    public static final double kPDriveVel = 3.7958;

    // Adjust as necessary for surroundings
    public static final double kMaxSpeedMetersPerSecond = .5;
    public static final double kMaxAccelerationMetersPerSecondSquared = .2;

    public static final double kvVoltSecondsPerRadian = 2.369;
    public static final double kaVoltSecondsSquaredPerRadian = 0.485;

    public static final double kRamseteB = 2; // dont change this and the val below
    public static final double kRamseteZeta = 0.7;

    public static final double TRACK_WIDTH_METERS = 0.57098;
    public static final double CIRCUMFERENCE = Math.PI * 6;
    public static final double GEAR_RATIO = 10.71;
}
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
