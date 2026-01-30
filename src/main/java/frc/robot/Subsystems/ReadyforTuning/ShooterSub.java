// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ReadyforTuning;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


//Shooter contains the feeder and flywheel systems
public class ShooterSub extends SubsystemBase {
  /** Creates a new Shooter. */


  private final SparkMax feederWheel1 = new SparkMax(18, MotorType.kBrushless);
  private final SparkMax feederWheel2 = new SparkMax(18, MotorType.kBrushless);
  private final SparkMax flywheel = new SparkMax(18, MotorType.kBrushless);
  RelativeEncoder flywheelEncoder = flywheel.getEncoder();
  PIDController FlywheelPID = new PIDController(0.001, 0, 0);

  public ShooterSub() {}

  public void FeederSetALL(double speed){
    feederWheel1.set(speed);
    feederWheel2.set(speed);
  }
  public void ShooterUnjam(){
    flywheel.set(-.5);
    feederWheel1.set(-.5);
    feederWheel2.set(-.5);
  }

    public void FlywheelSet(double speed){
    flywheel.set(speed);
  }

  public double getFlywheelRPM(){
    double flywheelRPM = flywheelEncoder.getVelocity();
    return flywheelRPM;
  } 
  public void FlywheelSetRPM(){
  //ADD SAFETY FEATURES TO TURRETSETPOINTS: PHYSICAL AND SOFTWARE LIMITS
  Double speed = FlywheelPID.calculate(getFlywheelRPM(), 3000); //high RPM setpoint
    flywheel.set(speed);  
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
