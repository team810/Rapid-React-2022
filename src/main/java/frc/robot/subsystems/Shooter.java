// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private Lightstrips LEDS;
  private CANSparkMax top, bottom;

  ShuffleboardTab tab;

  String color;

  private double topSpeed, bottomSpeed;

  private double distance; // inches, convert to whatever you need in the run command or shuffleboard
                           // command

  private double setPointTop, kPTop, kITop, kDTop, kFFTop;
  private double setPointBottom, kPBottom, kIBottom, kDBottom, kIzBottom, kFFBottom;
  private double kIz, kMinOutput, kMaxOutput; 

  NetworkTableEntry speedTop, speedBottom; 

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
    LEDS = new Lightstrips();

    this.top = new CANSparkMax(Constants.SHOOTER_TOP, MotorType.kBrushless);
    this.bottom = new CANSparkMax(Constants.SHOOTER_BOTTOM, MotorType.kBrushless);

    resetMotors();
    PIDinit();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateD();
    shuffleInit();
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
    this.LEDS.changeLEDColor(this.color = tv.getBoolean(true) ? "Green" : "Red");
    
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
    SmartDashboard.putNumber("Velocity Top(RPM)", this.top.getEncoder().getVelocity());
    SmartDashboard.putNumber("Velocity Top(%)", this.topSpeed);

    SmartDashboard.putNumber("Velocity Bottom(RPM)", this.bottom.getEncoder().getVelocity());
    SmartDashboard.putNumber("Velocity Bottom(%)", this.bottomSpeed);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

    SmartDashboard.putBoolean("Valid Target?", tv.getBoolean(false));
    SmartDashboard.putNumber("Limelight X", tx.getDouble(0));
    SmartDashboard.putNumber("Limelight Y", ty.getDouble(0));
    SmartDashboard.putNumber("Limelight Area", ta.getDouble(0));

    SmartDashboard.putNumber("Distance to target", this.distance);
  }

  private void PIDinit()
  {

    tab = Shuffleboard.getTab("Shooter System");
    //top
    setPointTop = tab.add("Set Speed (Top)", 5000).getEntry().getDouble(5000);
    speedTop = tab.add("Actual Speed (Top)", 0).getEntry();
    kPTop = tab.addPersistent("P (Top)", Constants.kPTop).getEntry().getDouble(0);
    kITop = tab.addPersistent("I (Top)", Constants.kITop).getEntry().getDouble(0);
    kDTop = tab.addPersistent("D (Top)", Constants.kDTop).getEntry().getDouble(0);
    kFFTop = tab.addPersistent("F (Top)", Constants.kFTop).getEntry().getDouble(0);

    //bottom
    setPointBottom = tab.add("Set Speed (Bottom)", 5000).getEntry().getDouble(5000);
    speedBottom = tab.add("Actual Speed (Bottom)", 0).getEntry();
    kPBottom = tab.addPersistent("P (Bottom)", Constants.kPBottom).getEntry().getDouble(0);
    kIBottom = tab.addPersistent("I (Bottom)", Constants.kIBottom).getEntry().getDouble(0);
    kDBottom = tab.addPersistent("D (Bottom)", Constants.kDBottom).getEntry().getDouble(0);
    kFFBottom = tab.addPersistent("F (Bottom)", Constants.kFBottom).getEntry().getDouble(0);

    top_pidcontroller = top.getPIDController();
    bottom_pidcontroller = bottom.getPIDController();

    kIz = 100;
    kMinOutput = -1;
    kMaxOutput = 1;
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