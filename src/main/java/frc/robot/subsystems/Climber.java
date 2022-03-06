// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
<<<<<<< HEAD
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
=======
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
<<<<<<< HEAD
  private CANSparkMax climberMotor;
  private double speed;
  private DoubleSolenoid leftHook, rightHook; 

  NetworkTableEntry speedRPM, speedPercent, climberPosition;

  ShuffleboardTab tab = Shuffleboard.getTab("Climber System");

  public Climber() {
    this.climberMotor = new CANSparkMax(Constants.CLIMBER_MOTOR, MotorType.kBrushless);
    this.leftHook = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.HOOKL_1, Constants.HOOKL_2);
    this.rightHook = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.HOOKR_1, Constants.HOOKR_2);

    motorReset();

    shuffleInit();
=======
  public CANSparkMax climberMotor;
  private double speed;

  private DoubleSolenoid sol;

  public Climber() {
    this.climberMotor = new CANSparkMax(Constants.CLIMB, MotorType.kBrushless);

    motorReset();
    climberMotor.setIdleMode(IdleMode.kCoast);

    sol = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 15);
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
<<<<<<< HEAD
    shuffleUpdate();
  }

  public void runClimber(double speed) {
    this.climberMotor.set(speed);
    this.speed = speed;
    while (this.climberMotor.getEncoder().getPosition() < Constants.CLIMBER_REVS) {}
    this.climberMotor.set(0);
  }

  public void toggleLeftHook(Value value)
  {
    leftHook.set(value);
  }

  public void toggleRightHook(Value value)
  {
    rightHook.set(value);
=======
    shuffleInit();

    //System.out.println(climberMotor.getEncoder().getPosition());



  }



  public void climbUp(double speed){
    if(climberMotor.getEncoder().getPosition() < -85){

      this.climberMotor.set(0);

  
      this.speed = speed;
  
        }
        else{
          this.climberMotor.set((speed));
  
        }

    if(climberMotor.getEncoder().getPosition() < 10){
      setPistons(false);

    }
  }

  public void climbDown(double speed){
    if(climberMotor.getEncoder().getPosition() > 50){

      this.climberMotor.set(0);
  
      this.speed = speed;
  
        }
        else{
          this.climberMotor.set((speed));
  
        }

        if(climberMotor.getEncoder().getPosition() < 10){
          setPistons(false);
    
        }
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
  }

  private void motorReset() {
    this.climberMotor.restoreFactoryDefaults();
<<<<<<< HEAD

=======
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
    this.climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void shuffleInit() {
<<<<<<< HEAD
    speedRPM = tab.add("Climber Velocity (RPM)", this.climberMotor.getEncoder().getVelocity()).getEntry();
    speedPercent = tab.add("Climber Velocity (%)", this.speed).getEntry();
    climberPosition = tab.add("Climber Position", this.climberMotor.getEncoder().getPosition()).getEntry();
  }

  public void shuffleUpdate() {
    speedRPM.setDouble(this.climberMotor.getEncoder().getVelocity());
    speedPercent.setDouble(this.speed);
    climberPosition.setDouble(this.climberMotor.getEncoder().getPosition());

  }
}
=======



    SmartDashboard.putNumber("Climber Position", -climberMotor.getEncoder().getPosition());

    SmartDashboard.putBoolean("Climber At Top", climberMotor.getEncoder().getPosition() < -85 ? true : false);    
    SmartDashboard.putBoolean("Hook Position", sol.get() == Value.kForward ? false : true);
  }

  public void togglePistons() {
    if (sol.get() == Value.kForward) {
      sol.set(Value.kReverse);
    } else {
      sol.set(Value.kForward);
    }
  }

  public void setPistons(boolean value) {
    if (value) {
      sol.set(Value.kReverse);
    } else {
      sol.set(Value.kForward);
    }
  }

  

}
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
