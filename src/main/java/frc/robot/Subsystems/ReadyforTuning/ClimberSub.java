// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ReadyforTuning;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSub extends SubsystemBase {
  /** Creates a new Climber. */
    private final SparkMax ClimberMotor = new SparkMax(13, MotorType.kBrushless);
     RelativeEncoder ClimberEncoder = ClimberMotor.getEncoder();
    double maxClimb = 250;
    double minClimb = 5;

  public ClimberSub() {}

  public void ClimberSetSpeed(double speed){
    System.out.println("CLIMBER SPEED: " + speed);
      getClimberPosition();
    if(speed > 0 && getClimberPosition() > maxClimb) {
      ClimberMotor.set(0);
    } else{
      ClimberMotor.set(speed);
    }

  }


  public void ClimberMotorUp(){
    double climberangle = ClimberEncoder.getPosition();
    SmartDashboard.putNumber("Climber Position", climberangle);
    if(maxClimb >= climberangle) ClimberMotor.set(1);

  }
  public void ClimberMotorDown(){
    double climberangle = ClimberEncoder.getPosition();
    SmartDashboard.putNumber("Climber Position", climberangle);
    if(minClimb <= climberangle) ClimberMotor.set(-1);
  }
  public void ClimberMotorStop(){
    ClimberMotor.set(0);
  }

  public double getClimberPosition(){
    double climberangle = ClimberEncoder.getPosition();
    SmartDashboard.putNumber("Climber Position", climberangle);
    return climberangle;
  }

  public void resetClimberPosition(){
    ClimberEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}