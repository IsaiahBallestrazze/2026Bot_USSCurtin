// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ReadyforTuning;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class TurretSub extends SubsystemBase {
  /** Creates a new TurretSub. */

//Turret is always on (toggle?) and pointing to our target position. 
//if it cant reach the target position it will go to the closest side 180 or 0 no matter the gyro rotation
//Turret 0 degrees is facing right side of robot, 90 is forward, 180 is left side of robot

private double targetX = 0; //X position of target ABSOLUTE except for red/blue side
private double targetY = 0; //Y position of target ABSOLUTE

private double robotX = 0; //X position of robot
private double robotY = 0; //Y position of robot


NetworkTable table = NetworkTableInstance.getDefault().getTable("Field"); //assuming this is right



      private final SparkMax turretMotor = new SparkMax(3, MotorType.kBrushless); // CAN ID
        final RelativeEncoder turretEncoder = turretMotor.getEncoder();

       PIDController TurretPID = new PIDController(0.3, 0, 0);

  public TurretSub() {}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Turret stuff below

public double CalculateRotationSpeed(double targetAngle, double turretCurrentAngle, PIDController pidController, double gyroAngle){
  double difference = (turretCurrentAngle - targetAngle) - gyroAngle;
  double setPoint;

  boolean method1 = ((difference <= 180) && (difference >= -180)) ? true: false;
  boolean method2 = (turretCurrentAngle >= 0) ? true: false;
  boolean method3 = (turretCurrentAngle < 0) ? true: false;

  if(method1){ // determines which equation to use to find shortest route go to https://www.desmos.com/calculator/tqoycuy5sz for a graph
    setPoint = -difference;
  } else if(method2){
    setPoint = -(difference-360);
  } else if(method3){
    setPoint = -(difference+360);
  } else{
    setPoint = 0;
  }

  double turretSpeed = pidController.calculate(0,setPoint);
  if(turretSpeed > 1) turretSpeed = 1;
  if(turretSpeed < -1) turretSpeed = -1;

    SmartDashboard.putNumber("GyroAngle", Math.toDegrees(gyroAngle));
    SmartDashboard.putNumber("Turret Current Angle", Math.toDegrees(turretCurrentAngle));
    SmartDashboard.putNumber("Target Angle Setpoint", Math.toDegrees(targetAngle));
    SmartDashboard.putNumber("Turret Speed", turretSpeed);
    SmartDashboard.putNumber("Setpoint", setPoint);

    SmartDashboard.putNumber("BotX", getFieldPositionX());
    SmartDashboard.putNumber("BotY", getFieldPositionY());

  return turretSpeed; //gives speed and direction
}

public double CalculateTargetAngle(double botX, double botY, double targetX, double targetY){ //finds the angle between the robot and the target point
  Double targetAngle = Math.atan2((targetY - botY),(targetX - botX)); // Use atan2 to handle all quadrants properly
  //point slope form to find line
  System.out.println("Target Angle (radians): " + targetAngle);
  return targetAngle;
}





public void resetTurretEncoder(){
  turretEncoder.setPosition(0);
  
}

public double getTurretPosition(){
  SmartDashboard.putNumber("Turret Encoder Here", Math.toRadians(turretEncoder.getPosition() / 19.5));
  return turretEncoder.getPosition();
}

public double getTurretRPM(){
  SmartDashboard.putNumber("Turret RPM", turretMotor.getEncoder().getVelocity());
  return turretMotor.getEncoder().getVelocity();
}

public double getTurretAngle(){ //assumes 0 is on right side
  double turretAngle = Math.toRadians(turretEncoder.getPosition() / 19.5); //gives 0 to pi
  //MUST CONVERT POSITION TO ANGLE
  return turretAngle;
}

  public void setTurretspeed(double speed) {
    // Convert degrees to motor rotations (assuming 1 rotation = 360 degrees)
  turretMotor.set(speed);  
}

public void setTurretAngle(double angle){ //assumes 0 is on right side //////////////////////////////////////////WAH WHY IN DEGREE LINMITS????
  if(angle < 0) angle = 0;
  if(angle > Math.PI) angle = Math.PI; //limits degree of movement

  double currentAngle = getTurretAngle();
  double turretSpeed = TurretPID.calculate(currentAngle, angle);
  setTurretspeed(turretSpeed);
}




public double getFieldPositionX() {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Field");

  double[] pose = table.getEntry("Robot").getDoubleArray(new double[3]);
  System.out.println("X Position: " + pose[0]);
  return pose[0];
}

public double getFieldPositionY() {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Field");

  double[] pose = table.getEntry("Robot").getDoubleArray(new double[3]);
  return pose[1];
}

public double getFieldPositionRotation() {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Field");

  double[] pose = table.getEntry("Robot").getDoubleArray(new double[3]);
  return pose[2];
}

public double SetFieldPosition(double x, double y, double rotation) { //gets replaced
  double[] pose = {x, y, rotation};

NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Field");



  table.getEntry("Robot").setDoubleArray(pose);
  System.out.println("Field Position Set to X: " + x + " Y: " + y + " Rotation: " + rotation);
  return pose[2];
}






































public PIDController getTurretPID(){
  return TurretPID;
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
