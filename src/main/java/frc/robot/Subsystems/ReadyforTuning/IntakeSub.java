// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ReadyforTuning;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// intake Sub contains the intake wheels, Agitators, and Intake Arm systems
public class IntakeSub extends SubsystemBase {
  /** Creates a new IntakeSub. */

  private final SparkMax IntakeWheelMotor = new SparkMax(18, MotorType.kBrushless);
  private final SparkMax AgitatorMotor = new SparkMax(18, MotorType.kBrushless);
  private final SparkMax IntakeArmMotor = new SparkMax(18, MotorType.kBrushless);
  RelativeEncoder IntakeArmEncoder = IntakeArmMotor.getEncoder();
  PIDController ArmPID = new PIDController(0.001, 0, 0);

  
    public IntakeSub() {}


  public void IntakeWheelSet(double speed){
    IntakeWheelMotor.set(speed);
  }
  public void AgitatorSet(double speed){
    AgitatorMotor.set(speed);
  }
  public void IntakeArmSet(double speed){ //ADD SAFETY LIMITS LATER
    IntakeArmMotor.set(speed);
  }
  public double getIntakeArmPosition(){ //MAY WANT TO CONVERT TO DEGREES LATER
    double IntakeArmAngle = IntakeArmEncoder.getPosition();
    return IntakeArmAngle;
  }

  public void ArmSetpointBottom(){
  //ADD SAFETY FEATURES TO TURRETSETPOINTS: PHYSICAL AND SOFTWARE LIMITS
  Double speed = ArmPID.calculate(getIntakeArmPosition(), 90); //bottom position assuming 0 straight up inside frame
    IntakeArmMotor.set(speed);  
}

  public void ArmSetpointTop(){
  //ADD SAFETY FEATURES TO TURRETSETPOINTS: PHYSICAL AND SOFTWARE LIMITS
  Double speed = ArmPID.calculate(getIntakeArmPosition(), 0); //top position assuming 0 straight up inside frame
    IntakeArmMotor.set(speed);  
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
