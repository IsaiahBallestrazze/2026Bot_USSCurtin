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

  private final SparkMax mecanumWheels = new SparkMax(12, MotorType.kBrushed);
  private final SparkMax greenWheels = new SparkMax(14, MotorType.kBrushless);

  public ShooterSub() {}

  public void ShooterGroup(double mecanumSpeed, double greenWheelSpeed){ 
    mecanumWheels.set(mecanumSpeed);
    greenWheels.set(-greenWheelSpeed);
  }



  public void ShooterUnjam(){
    greenWheels.set(-.5);
    mecanumWheels.set(-.5);
  }

    public void mecanumSet(double speed) {
    mecanumWheels.set(speed);
}

    public void greenWheelSet(double speed) {
    greenWheels.set(-speed);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
