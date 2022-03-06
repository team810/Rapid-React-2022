// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

<<<<<<< HEAD
import edu.wpi.first.networktables.NetworkTableInstance;
=======
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ColorSensorV3;

>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
<<<<<<< HEAD
  private Spark intakeMotor;
  private DoubleSolenoid intakeSol;

  /** Creates a new Intake. */
  public Intake() {
    this.intakeMotor = new Spark(Constants.INTAKE_MOTOR);
    this.intakeSol = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_1, Constants.INTAKE_SOLENOID_2);
=======

  private Spark intakeMotor;
  private DoubleSolenoid sol;
  private double speed;

  //private Spark feederMotor;



  //private final I2C.Port i2cPort = I2C.Port.kOnboard;

  //private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new Spark(Constants.INTAKE);

   // feederMotor = new Spark(Constants.FEEDER);

    sol = new DoubleSolenoid(PneumaticsModuleType.REVPH, 12, 13);

>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
  }

  @Override
  public void periodic() {
<<<<<<< HEAD
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed, int state) {
    this.intakeMotor.set(speed);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(state);
=======



  // ejectBall();
  }


  public void toggleSolenoid() {
    if(sol.get() == Value.kForward){
      sol.set(Value.kReverse);
    }else{
      sol.set(Value.kForward);
    }
  }

  public void setIntake(boolean state){
    if(state){

      sol.set(Value.kForward);
      intakeMotor.set(1);
    }

    else{
      sol.set(Value.kReverse);
      intakeMotor.set(0);
    }

  }

  public void run(double speed){
    intakeMotor.set(speed);
    
  }

  public void runFeeder(double speed){
   // feederMotor.set(speed);
  }

  //public void runIntakeFeed(double speed){
    //intakeMotor.set(speed);
    //feederMotor.set(speed);
  //}

  /*public boolean getCorrectBall() {

    Color detectedColor = m_colorSensor.getColor();

    DriverStation.getAlliance();
	if (DriverStation.getAlliance() == Alliance.Red) {
      if (detectedColor.red > .5) {
        return true;
      } else {
        return false;
      }
    } else {
		DriverStation.getAlliance();
		if (DriverStation.getAlliance() == Alliance.Blue) {
		  if (detectedColor.blue > .5) {
		    return true;
		  } else {
		    return false;
		  }
		}
	}
    return false;
  }

  public void ejectBall(){
    if(!getCorrectBall()){
   //   m_lights.changeLEDColor("R");
      setIntake(false, -.7);
    }
  }*/

 private void shuffleInit() {
    //SmartDashboard.putNumber("Intake Velcocity (RPM)", this.intakeMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Intake Velcocity (%)", this.speed);

    //SmartDashboard.putNumber("Intake Position", this.intakeMotor.getEncoder().getPosition());

    SmartDashboard.putBoolean("Solenoid State", sol.get() == Value.kForward ? true : false);
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
  }
} 
  

<<<<<<< HEAD
  public void toggleIntakeSolenoid(Value value)
  {
    intakeSol.set(value);
  }
}
=======
>>>>>>> 217a4f0e72de65d4eeb3c17ad44a7bd46b57666f
