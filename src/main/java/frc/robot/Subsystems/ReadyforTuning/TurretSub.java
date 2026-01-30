// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ReadyforTuning;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class TurretSub extends SubsystemBase {
  /** Creates a new TurretSub. */

  //3 setpoints on buttonbox for turret to got to
  //assume 0 is robot turned all the way to the right ->
  // button 1 sends it to 45 degrees
  // button 2 sends it to 90 degrees
  // button 3 sends it to 135 degrees <-
  //auto adjust from limelight gives around +- 27 degrees of adjustment
  // turret auto adjsuts from setpoints based on limelight data to stay on target

  //holding button calls setpoint and auto adjust script

  //to know the position of the turret we must get the relative encoder position and convert it to an angle.

      private final SparkMax turretMotor = new SparkMax(10, MotorType.kBrushless); // CAN ID
        final RelativeEncoder turretEncoder = turretMotor.getEncoder();

       PIDController TurretPID = new PIDController(0.001, 0, 0);

  public TurretSub() {

  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Turret stuff below

  public void setTurretspeed(double speed) {
    // Convert degrees to motor rotations (assuming 1 rotation = 360 degrees)
  turretMotor.set(speed);  
}

public void setpointRight(){
  //ADD SAFETY FEATURES TO TURRETSETPOINTS: PHYSICAL AND SOFTWARE LIMITS
  Double turretSpeed = TurretPID.calculate(getTurretAngle(), 45 + getTx()); //right position assuming 0 is right side
    turretMotor.set(turretSpeed);  
}
public void setpointLeft(){
  Double turretSpeed = TurretPID.calculate(getTurretAngle(), 135 + getTx()); //left position assuming 0 is right side
    turretMotor.set(turretSpeed);  
}
public void setpointCenter(){
  Double turretSpeed = TurretPID.calculate(getTurretAngle(), 90 + getTx()); //center position assuming 0 is right side
    turretMotor.set(turretSpeed);  
}

public double getTurretAngle(){ //assumes 0 is on right side
  double turretAngle = turretEncoder.getPosition(); // in rotations
  //MUST CONVERT POSITION TO ANGLE
  return turretAngle;
}

//TURRET STUFF ABOVE
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//VISION STUFF BELOW

public double getTx() {
  return LimelightHelpers.getTX(""); //X position away from target position
}

public double getTy() {
  return LimelightHelpers.getTY(""); //Y position away from target position
}

public double getTa() {
  return LimelightHelpers.getTA("");
}

public boolean getTV() {
  return LimelightHelpers.getTV("");
}

public double getTXNC() {
  return LimelightHelpers.getTXNC(""); //NC MEANS ITS THE RAW OUTPUT
}

public double[] getBotPoseBlue() {
    return LimelightHelpers.getBotPose_wpiBlue(""); //POSE IS USED TO FIND THE POSITION OF THE ROBOT ON THE FIELD ALWAYS USE BLUE
}

public double[] getBotPoseRed() {
    return LimelightHelpers.getBotPose_wpiRed("");
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
