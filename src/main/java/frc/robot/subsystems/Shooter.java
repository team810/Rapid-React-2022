// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax topMotor, bottomMotor;

  private double topSpeed, bottomSpeed;

  private double distance; // inches, convert to whatever you need in the run command or shuffleboard
                           // command

  private int goalHeight = 96; // inches
  private int limelightHeight = 48; // inches
  private int limelightAngle = 45; // degrees

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  /** Creates a new Shooter. */
  public Shooter() {
    this.topMotor = new CANSparkMax(Constants.SHOOTER_TOP, MotorType.kBrushless);
    this.bottomMotor = new CANSparkMax(Constants.SHOOTER_BOTTOM, MotorType.kBrushless);

    resetMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateD();
    shuffleInit();
  }

  public void runShooter(double topSpeed, double bottomSpeed) {
    this.topMotor.set(-topSpeed);
    this.bottomMotor.set(bottomSpeed);

    this.topSpeed = topSpeed;
    this.bottomSpeed = bottomSpeed;
  }

  private void resetMotors() {
    this.topMotor.restoreFactoryDefaults();
    this.bottomMotor.restoreFactoryDefaults();
    /*
     * this decreases the time between shots by leaving the motors at a higher speed
     * when not in use
     */
    this.topMotor.setIdleMode(IdleMode.kCoast);
    this.bottomMotor.setIdleMode(IdleMode.kCoast);
  }

  private void shuffleInit() {
    SmartDashboard.putNumber("Velocity Top(RPM)", this.topMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Velocity Top(%)", this.topSpeed);

    SmartDashboard.putNumber("Velocity Bottom(RPM)", this.bottomMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Velocity Bottom(%)", this.bottomSpeed);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

    SmartDashboard.putNumber("Limelight X", Constants.tx.getDouble(0));
    SmartDashboard.putNumber("Limelight Y", Constants.ty.getDouble(0));
    SmartDashboard.putNumber("Limelight Area", Constants.ta.getDouble(0));

    SmartDashboard.putNumber("Distance to target", this.distance);
  }

<<<<<<< HEAD
  private void updateD() {
=======
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
    this.tab = Shuffleboard.getTab("Shooter System");
    //top
    this.setPointTop = tab.add("Set Speed (Top)", 1000).getEntry();
    this.speedTop = tab.add("Actual Speed (Top)", 0).getEntry();
    this.kPTop = tab.addPersistent("P (Top)", 0).getEntry();
    this.kITop = tab.addPersistent("I (Top)", 0).getEntry();
    this.kDTop = tab.addPersistent("D (Top)", 0).getEntry();
    this.kFFTop = tab.addPersistent("FF (Top)", 0).getEntry();

    //bottom
    this.setPointBottom = tab.add("Set Speed (Bottom)", 1000).getEntry();
    this.speedBottom = tab.add("Actual Speed (Bottom)", 0).getEntry();
    this.kPBottom = tab.addPersistent("P (Bottom)", 0).getEntry();
    this.kIBottom = tab.addPersistent("I (Bottom)", 0).getEntry();
    this.kDBottom = tab.addPersistent("D (Bottom)", 0).getEntry();
    this.kFFBottom = tab.addPersistent("FF (Bottom)", 0).getEntry();

    this.top_pidcontroller = top.getPIDController();
    this.bottom_pidcontroller = bottom.getPIDController();

    this.kIz = 100;
    this.kMinOutput = -1;
    this.kMaxOutput = 1;

    top_pidcontroller.setP(kPTop.getDouble(0));
    top_pidcontroller.setI(kITop.getDouble(0));
    top_pidcontroller.setD(kDTop.getDouble(0));
    top_pidcontroller.setFF(1/5600);
    top_pidcontroller.setIZone(kIz);
    top_pidcontroller.setOutputRange(kMinOutput, kMaxOutput);

    bottom_pidcontroller.setP(kPBottom.getDouble(0));
    bottom_pidcontroller.setI(kIBottom.getDouble(0));  
    bottom_pidcontroller.setD(kDBottom.getDouble(0));
    bottom_pidcontroller.setFF(1/5600);
    bottom_pidcontroller.setIZone(kIz);
    bottom_pidcontroller.setOutputRange(kMinOutput, kMaxOutput);
  }

  private void resetMotors() {
    this.top.restoreFactoryDefaults();
    this.bottom.restoreFactoryDefaults();
>>>>>>> 6a78b70ad61903f26fbb8eff7f6774c09cf20599
    /*
     * d = (h2-h1) / tan(a1+a2) d = (hieght of the limelight from ground minus the
     * heigth of the field goal inches) over (tan(angle of lens to goal + angle of
     * lens from bottom of camera))
     */
    // Collect data and run linear regression for motor power to distance linear
    // relationship to implement to shoot command
    this.distance = (this.goalHeight - this.limelightHeight)
        / Math.tan(Math.toRadians(this.limelightAngle + Constants.ty.getDouble(0)));
  }
}