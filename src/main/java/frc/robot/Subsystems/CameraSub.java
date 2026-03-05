// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSub extends SubsystemBase {
  /** Creates a new Camera. */
  public CameraSub() {
    StartCamera();
  }

public void StartCamera(){
UsbCamera RobotEye = CameraServer.startAutomaticCapture(0);
//UsbCamera RobotEye2 = CameraServer.startAutomaticCapture(1);

RobotEye.setResolution(640, 480);
RobotEye.setFPS(60);
// RobotEye2.setResolution(1280, 720);
// RobotEye2.setFPS(30);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}