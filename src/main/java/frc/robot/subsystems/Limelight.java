// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

  public NetworkTableEntry ledMode = Constants.ledMode;
  public NetworkTableEntry camMode = Constants.camMode;

  public NetworkTableEntry pipeline = Constants.pipeline;
  public NetworkTableEntry stream = Constants.stream;




  /** Creates a new Limelight. */
  public Limelight() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLightMode(boolean state){
    if(state){
      lightOn();
    }  
    else{
      lightOff();
    }
  }

  public void visionMode() {
    camMode.setNumber(0); // sets camera to vision processing mode
    pipeline.setNumber(0);
  }

  public void driverMode() {
    camMode.setNumber(1); // sets camera to driving mode
    pipeline.setNumber(0);
  }

  

  /**
   * Forces on light
   */
  public void lightOn() {
    ledMode.setNumber(3);
  }

  /**
   * Forces off light
   */
  public void lightOff() {
    ledMode.setNumber(1);
  }

  /**
   * Changes light settings according to how vision pipeline is set
   */
  public void lightAuto() {
    ledMode.setNumber(0);
  }
    
  /**
   * Checks if there is no valid targets, which is then sent to isFinsished() 
   */
  // public boolean noValidTarget() {
  //   if (validTarget == 0) {
  //     SmartDashboard.putBoolean("Target", false);
  //     return true; 

  //   } else {
  //     SmartDashboard.putBoolean("Target", true);
  //     return false;
      
  //   }
  // }

  public boolean isValidTarget(){
    if(Constants.tv.getDouble(0.0) == 1){
      return true;
    }
    else{
      return false;
    }
  }

  public double getRPM(){
    return 0;
  }

  public double getAngle(){
    return Constants.ty.getDouble(0.0);
  }

  

  // public double getDistance(){
  //   double a = this.angle + Constants.ty.getDouble(0.0); 
  //   System.out.println(a);
  //   double distToTarget = (98.75 - 19) / Math.tan(Math.toRadians(a));
  //   return distToTarget;
  // }
}
