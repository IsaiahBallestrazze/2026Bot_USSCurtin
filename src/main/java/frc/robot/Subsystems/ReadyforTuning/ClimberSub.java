// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ReadyforTuning;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSub extends SubsystemBase {
  /** Creates a new Climber. */
    private final SparkMax ClimberMotor = new SparkMax(18, MotorType.kBrushless);
     RelativeEncoder ClimberEncoder = ClimberMotor.getEncoder();
    double maxClimb = 200;
    double minClimb = -200;

  public ClimberSub() {}

  public void ClimberMotorUp(){
    double climberangle = ClimberEncoder.getPosition();
    if(maxClimb >= climberangle) ClimberMotor.set(.5);

  }
  public void ClimberMotorDown(){
    double climberangle = ClimberEncoder.getPosition();
    if(minClimb <= climberangle) ClimberMotor.set(-.5);
  }
  public void ClimberMotorStop(){
    ClimberMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}