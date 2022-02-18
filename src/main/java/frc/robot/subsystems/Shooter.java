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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkMax top, bottom;

  ShuffleboardTab tab = Shuffleboard.getTab("Shooter System");

  // private double setPointTop, kPTop, kITop, kDTop, kFFTop;
  // private double setPointBottom, kPBottom, kIBottom, kDBottom, kFFBottom;
  private double kIz, kMinOutput, kMaxOutput;

  SparkMaxPIDController top_pidcontroller, bottom_pidcontroller;

  private double topSpeed, bottomSpeed;

  private double distance; // inches, convert to whatever you need in the run command or shuffleboard
                           // command
  NetworkTableEntry setSpeedTop, setPTop, setITop, setDTop, setFTop,
      setSpeedBottom, setPBottom, setIBottom, setDBottom, setFBottom;

  private int goalHeight = 96; // inches
  private int limelightHeight = 48; // inches
  private int limelightAngle = 45; // degrees

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  private NetworkTableEntry tv = table.getEntry("tv");

  private NetworkTableEntry shooterTopVRPM = tab.add("Shooter Velcocity (RPM)", 0)
      .getEntry();
  private NetworkTableEntry shooterTopVP = tab.add("Shooter Velcocity (%)", this.topSpeed)
      .getEntry();

  private NetworkTableEntry shooterBotVRPM = tab.add("Shooter Vel Bot (RPM)", 0)
      .getEntry();
  private NetworkTableEntry shooterBotVP = tab.add("Shooter Vel Bot (%)", this.bottomSpeed)
      .getEntry();

  /** Creates a new Shooter. */
  public Shooter() {
    this.top = new CANSparkMax(Constants.SHOOTER_TOP, MotorType.kBrushless);
    this.bottom = new CANSparkMax(Constants.SHOOTER_BOTTOM, MotorType.kBrushless);

    resetMotors();
    shuffleInit();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateD();
    shuffleUpdate();
  }

  public void toggleLimelightLight(int value) {
    table.getEntry("ledMode").setNumber(value);
  }

  public void toggleLimelightCamMode(int value) {
    table.getEntry("camMode").setNumber(value);
  }

  public void runShooter(double topSpeed, double bottomSpeed) {
    this.top.set(-topSpeed);
    this.bottom.set(bottomSpeed);

    this.topSpeed = topSpeed;
    this.bottomSpeed = bottomSpeed;
  }

  public void runTop() {
    top_pidcontroller.setP(setPTop.getDouble(0));
    top_pidcontroller.setI(setITop.getDouble(0));
    top_pidcontroller.setD(setDTop.getDouble(0));
    top_pidcontroller.setFF(setFTop.getDouble(0));
    top_pidcontroller.setIZone(kIz);
    top_pidcontroller.setOutputRange(kMinOutput, kMaxOutput);

    top_pidcontroller.setReference(setSpeedTop.getDouble(0), ControlType.kVelocity);
    shooterTopVRPM.setDouble(top.getEncoder().getVelocity());
  }

 
  public void runBottom() {
    bottom_pidcontroller.setP(setPBottom.getDouble(0));
    bottom_pidcontroller.setI(setIBottom.getDouble(0));
    bottom_pidcontroller.setD(setDBottom.getDouble(0));
    bottom_pidcontroller.setFF(setFBottom.getDouble(0));
    bottom_pidcontroller.setIZone(kIz);
    bottom_pidcontroller.setOutputRange(kMinOutput, kMaxOutput);

    bottom_pidcontroller.setReference(setSpeedBottom.getDouble(0), ControlType.kVelocity);
    shooterBotVRPM.setDouble(top.getEncoder().getVelocity());
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
    Shuffleboard.selectTab("Shooter System");
    setSpeedTop = tab.add("Set Speed Top", 1000).getEntry();
    setSpeedBottom = tab.add("Set Speed Bottom", 1000).getEntry();
    setPTop = tab.addPersistent("P top", 0).getEntry();
    setITop = tab.addPersistent("I top", 0).getEntry();
    setDTop = tab.addPersistent("D top", 0).getEntry();
    setFTop = tab.addPersistent("F top", 0).getEntry();
    setPBottom = tab.addPersistent("P", 0).getEntry();
    setIBottom = tab.addPersistent("I", 0).getEntry();
    setDBottom = tab.addPersistent("D", 0).getEntry();
    setFBottom = tab.addPersistent("F", 0).getEntry();
  } 

  private void shuffleUpdate() {
    shooterTopVRPM.setDouble(this.top.getEncoder().getVelocity());
    shooterTopVP.setDouble(this.topSpeed);

    shooterBotVRPM.setDouble(this.bottom.getEncoder().getVelocity());
    shooterBotVP.setDouble(this.bottomSpeed);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

    SmartDashboard.putBoolean("Valid Target?", tv.getBoolean(false));
    SmartDashboard.putNumber("Limelight X", tx.getDouble(0));
    SmartDashboard.putNumber("Limelight Y", ty.getDouble(0));
    SmartDashboard.putNumber("Limelight Area", ta.getDouble(0));

    SmartDashboard.putNumber("Distance to target", this.distance);
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