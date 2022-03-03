// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax climberMotor;
  private double speed;

  private DoubleSolenoid sol;

  public Climber() {
    this.climberMotor = new CANSparkMax(Constants.CLIMB, MotorType.kBrushless);

    motorReset();

    sol = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 15);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleInit();
  }

  public void runClimber(double speed) {
   // if(DriverStation.getMatchTime() < 45){

    
    this.climberMotor.set(speed);
    
    this.speed = speed;

    System.out.println(speed);
  
    //}
  }

  private void motorReset() {
    this.climberMotor.restoreFactoryDefaults();
    this.climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void shuffleInit() {
    SmartDashboard.putNumber("Climber Velocity (RPM)", this.climberMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Climber Velocity (%)", this.speed);

    SmartDashboard.putNumber("Climber Position", this.climberMotor.getEncoder().getPosition());
  }

  public void togglePistons(){
    if(sol.get() == Value.kForward){
      sol.set(Value.kReverse);
    }else{
      sol.set(Value.kForward);
    }
  }

}