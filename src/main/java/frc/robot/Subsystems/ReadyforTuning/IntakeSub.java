// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ReadyforTuning;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// intake Sub contains the intake wheels, Agitators, and Intake Arm systems
public class IntakeSub extends SubsystemBase {
  /** Creates a new IntakeSub. */

  private final SparkMax IntakeWheelMotor = new SparkMax(11, MotorType.kBrushless); //roller NOT MECANUM
  private final SparkMax AgitatorMotor = new SparkMax(10, MotorType.kBrushed);
  private final SparkMax IntakeArmMotor = new SparkMax(9, MotorType.kBrushed);
  //RelativeEncoder IntakeArmEncoder = IntakeArmMotor.getEncoder();
  //PIDController ArmPID = new PIDController(0.001, 0, 0);

  
    public IntakeSub() {}

  public void IntakeGroup(double rollerSpeed, double agitatorSpeed){
    IntakeWheelMotor.set(rollerSpeed);
    AgitatorMotor.set(agitatorSpeed);
    //ArmSetpointBottom();
  }

  public void IntakeWheelSet(double speed){
    IntakeWheelMotor.set(speed);
  }
  public void AgitatorSet(double speed){
    AgitatorMotor.set(speed);
  }
  public void IntakeArmSet(double speed){ //ADD SAFETY LIMITS LATER
    IntakeArmMotor.set(speed);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }




}


