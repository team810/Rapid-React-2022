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
  public CANSparkMax top, bottom;
  public double distance;
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  //Heights for vision (inches)
  private int goalHeight = 96;
  private int limelightHeight = 48;

  /** Creates a new Shooter. */
  public Shooter() {
    top = new CANSparkMax(Constants.SHOOTER_TOP, MotorType.kBrushless);
    bottom = new CANSparkMax(Constants.SHOOTER_BOTTOM, MotorType.kBrushless);

    /*
     * this decreases the time between shots
     * by leaving the motors at a higher speed when not in use
     * 
     */
    top.setIdleMode(IdleMode.kCoast);
    bottom.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateD();
    shuffleInit();
  }

  public void runShooter(double topSpeed, double bottomSpeed) {
    top.set(-topSpeed);
    bottom.set(bottomSpeed);
  }

  private void shuffleInit()
  {
    SmartDashboard.putNumber("Velocity Top(RPM)", top.getEncoder().getVelocity());
    SmartDashboard.putNumber("Velocity Bottom(RPM)", top.getEncoder().getVelocity());

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

    SmartDashboard.putNumber("Limelight X", tx.getDouble(0));
    SmartDashboard.putNumber("Limelight Y", ty.getDouble(0));
    SmartDashboard.putNumber("Limelight Area", ta.getDouble(0));

    SmartDashboard.putNumber("Distance to target", this.distance);
  }

  private void updateD()
  {
    /*
    * d = (h2-h1) / tan(a1+a2)
    * d = (hieght of the limelight from ground minus the heigth of the field goal inches) over
    *     (tan(angle of lens to goal + angle of lens from bottom of camera))
    */
    //Or manually find the distances yourself to set this relatively
    this.distance = (this.goalHeight-this.limelightHeight) / Math.tan( Math.toRadians( 45 + ty.getDouble(0)) );
  }
}