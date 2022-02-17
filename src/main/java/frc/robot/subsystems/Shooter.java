// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  // private Lightstrips LEDS;
  private CANSparkMax top, bottom;

  ShuffleboardTab tab;

  String color;

  private double topSpeed, bottomSpeed;

  private double distance; // inches, convert to whatever you need in the run command or shuffleboard
                           // command

  private double setPointTop, kPTop, kITop, kDTop, kFFTop;
  private double setPointBottom, kPBottom, kIBottom, kDBottom, kFFBottom;
  private double kIz, kMinOutput, kMaxOutput; 

  NetworkTableEntry speedTop, speedBottom; 

  NetworkTableEntry topVelRPM, topVelPercent, 
                    bottomVelRPM, bottomVelPercent,
                    targetValidity, limelightX, limelightY, limelightArea, targetDistance; 

  SparkMaxPIDController top_pidcontroller, bottom_pidcontroller; 

  private int goalHeight = 96; // inches
  private int limelightHeight = 48; // inches
  private int limelightAngle = 45; // degrees

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  private NetworkTableEntry tv = table.getEntry("tv");

  /** Creates a new Shooter. */
  public Shooter() {
    // LEDS = new Lightstrips();

    this.top = new CANSparkMax(Constants.SHOOTER_TOP, MotorType.kBrushless);
    this.bottom = new CANSparkMax(Constants.SHOOTER_BOTTOM, MotorType.kBrushless);

    resetMotors();
    PIDinit();
    shuffleInit();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateD();
    shuffleUpdate();
  }

  public void toggleLimelightLight(int value)
  {
    table.getEntry("ledMode").setNumber(value);
  }
  public void toggleLimelightCamMode(int value)
  {
    table.getEntry("camMode").setNumber(value);
  }

  public void runShooter(double topSpeed, double bottomSpeed) {
    // Red for not valid target, Green for valid target
    // this.LEDS.changeLEDColor(this.color = tv.getBoolean(true) ? "Green" : "Red");
    
    this.top.set(-topSpeed);
    this.bottom.set(bottomSpeed);

    this.topSpeed = topSpeed;
    this.bottomSpeed = bottomSpeed;
  }


  public void runTop()
  {
    top_pidcontroller.setP(kPTop);
    top_pidcontroller.setI(kITop);
    top_pidcontroller.setD(kDTop);
    top_pidcontroller.setFF(kFFTop);
    top_pidcontroller.setIZone(kIz);
    top_pidcontroller.setOutputRange(kMinOutput, kMaxOutput);

    top_pidcontroller.setReference(setPointTop, ControlType.kVelocity);
    speedTop.setDouble(top.getEncoder().getVelocity());
  }

  public void runBottom()
  {

    bottom_pidcontroller.setP(kPBottom);
    bottom_pidcontroller.setI(kIBottom);
    bottom_pidcontroller.setD(kDBottom);
    bottom_pidcontroller.setFF(kFFBottom);
    bottom_pidcontroller.setIZone(kIz);
    bottom_pidcontroller.setOutputRange(kMinOutput, kMaxOutput);

    bottom_pidcontroller.setReference(setPointBottom, ControlType.kVelocity);
    speedBottom.setDouble(top.getEncoder().getVelocity());
  }

  private void resetMotors() {
    this.top.restoreFactoryDefaults();
    this.bottom.restoreFactoryDefaults();
    /*
     * this decreases the time between shots
     * by leaving the motors at a higher speed when not in use
     */
    this.top.setIdleMode(IdleMode.kCoast);
    this.bottom.setIdleMode(IdleMode.kCoast);
  }

  private void shuffleInit() {
    topVelRPM = tab.add("Velocity Top(RPM)", this.top.getEncoder().getVelocity()).getEntry();
    topVelPercent = tab.add("Velocity Top(%)", this.topSpeed).getEntry();

    bottomVelRPM = tab.add("Velocity Bottom(RPM)", this.bottom.getEncoder().getVelocity()).getEntry();
    bottomVelPercent = tab.add("Velocity Bottom(%)", this.bottomSpeed).getEntry();

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

    targetValidity = tab.add("Valid Target?", tv.getBoolean(false)).getEntry();
    limelightX = tab.add("Limelight X", tx.getDouble(0)).getEntry();
    limelightY = tab.add("Limelight Y", ty.getDouble(0)).getEntry();
    limelightArea = tab.add("Limelight Area", ta.getDouble(0)).getEntry();

    targetDistance = tab.add("Distance to target", this.distance).getEntry();
  }

  private void shuffleUpdate() {
    topVelRPM.setDouble(this.top.getEncoder().getVelocity());
    topVelPercent.setDouble(this.topSpeed);

    bottomVelRPM.setDouble(this.bottom.getEncoder().getVelocity());
    bottomVelPercent.setDouble(this.bottomSpeed);

    targetValidity.setBoolean(tv.getBoolean(false));
    limelightX.setDouble(tx.getDouble(0));
    limelightY.setDouble(ty.getDouble(0));
    limelightArea.setDouble(ta.getDouble(0));

    targetDistance.setDouble(this.distance);
  }

  private void PIDinit()
  {

    /* 
    For next year, I'd like to explain PID tuning so no one gets confused as much.
    Upon pressing a motor with an external force, like a ball, or just in normal motion,
    the motor's voltage cannot be perfect; the motor will slow or speed up slightly.
    To fix this, we use PID tuning: this accounts for the error and makes sure that the motor
    is always running at the same speed, no matter what happens to it. 

    P: Proportional - This is the error at present in the motor.
    I: Integral - This is the accumulate error of the motor over a certain zone.
    D: Derivative - This is the rate of change of the error in the motor at present.
    F or FF: Feed Forward - This is the prediction of the error of the motor, and this is 
    fed into the motor when you run PID tuning and is usually set as 1/(RPM wanted) or 1/(max RPM).  

    The actual math doesn't matter that much, as the code does it for us. You can look it up 
    if you'd like, though it's difficult to understand and there's no real reason to do it.
    I personally, did not go the length to fully understand the math of the PID equation, though 
    I understood the concepts it was built upon and the calculus it used. 

    With these three values (found experimentally), we can use PID tuning to make sure our motors 
    run the way we want to, and our drivetrain always goes the straightest it possibly can.

    This is only a short account of PID tuning, however, and, though next year's coders
    will surely have to do more research and awful code parsing, I just wanted to get the basic gist
    across so next year isn't as confused as I was.

    -Anagh
    */

    this.tab = Shuffleboard.getTab("Shooter System");
    //top
    this.setPointTop = tab.add("Set Speed (Top)", 5000).getEntry().getDouble(5000);
    this.speedTop = tab.add("Actual Speed (Top)", 0).getEntry();
    this.kPTop = tab.addPersistent("P (Top)", Constants.kPTop).getEntry().getDouble(0);
    this.kITop = tab.addPersistent("I (Top)", Constants.kITop).getEntry().getDouble(0);
    this.kDTop = tab.addPersistent("D (Top)", Constants.kDTop).getEntry().getDouble(0);
    this.kFFTop = tab.addPersistent("F (Top)", Constants.kFTop).getEntry().getDouble(0);

    //bottom
    this.setPointBottom = tab.add("Set Speed (Bottom)", 5000).getEntry().getDouble(5000);
    this.speedBottom = tab.add("Actual Speed (Bottom)", 0).getEntry();
    this.kPBottom = tab.addPersistent("P (Bottom)", Constants.kPBottom).getEntry().getDouble(0);
    this.kIBottom = tab.addPersistent("I (Bottom)", Constants.kIBottom).getEntry().getDouble(0);
    this.kDBottom = tab.addPersistent("D (Bottom)", Constants.kDBottom).getEntry().getDouble(0);
    this.kFFBottom = tab.addPersistent("F (Bottom)", Constants.kFBottom).getEntry().getDouble(0);

    this.top_pidcontroller = top.getPIDController();
    this.bottom_pidcontroller = bottom.getPIDController();

    this.kIz = 100;
    this.kMinOutput = -1;
    this.kMaxOutput = 1;
  }

  private void updateD() {
    /*
     * d = (h2-h1) / tan(a1+a2)
     * d = (hieght of the limelight from ground minus the heigth of the field goal
     * inches) over
     * (tan(angle of lens to goal + angle of lens from bottom of camera))
     */
    // Collect data and run linear regression for motor power to distance linear
    // relationship to implement to shoot command
    this.distance = (this.goalHeight - this.limelightHeight)
        / Math.tan(Math.toRadians(this.limelightAngle + ty.getDouble(0)));
  }
}