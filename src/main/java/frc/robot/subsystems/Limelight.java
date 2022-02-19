// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.temporal.IsoFields;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase {

  public NetworkTableEntry ledMode = Constants.ledMode;
  public NetworkTableEntry camMode = Constants.camMode;

  public NetworkTableEntry pipeline = Constants.pipeline;
  public NetworkTableEntry stream = Constants.stream;

  RobotContainer m_container;


  Lightstrips m_lights;

  HttpCamera feed;

  /** Creates a new Limelight. */
  public Limelight() {

    m_lights = new Lightstrips();

    feed = new HttpCamera("Limelight", "http://limelight.local:5800/");
    CameraServer.getInstance().startAutomaticCapture(feed);


  }

  @Override
  public void periodic() {
    if(isValidTarget()){
    if(Constants.tx.getDouble(0.0) > 0){
      m_container.GP.setRumble(RumbleType.kRightRumble, .5);
    }
    if(Constants.tx.getDouble(0.0) < 0){
      m_container.GP.setRumble(RumbleType.kRightRumble, .5);
    }
    if(Math.abs(Constants.tx.getDouble(0.0)) < 5){
      m_container.GP.setRumble(RumbleType.kRightRumble, .1);
      m_container.GP.setRumble(RumbleType.kLeftRumble, .1);

    }
  }
    



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

  public void changeColorOnValidTarget(){
    if(isValidTarget()){
      m_lights.changeLEDColor("G");
    }
    else{
      m_lights.changeLEDColor("B");
    }
  }

  


  // public double getDistance(){
  //   double a = this.angle + Constants.ty.getDouble(0.0); 
  //   System.out.println(a);
  //   double distToTarget = (98.75 - 19) / Math.tan(Math.toRadians(a));
  //   return distToTarget;
  // }
}
