// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.ReadyforTuning.ClimberSub;
import frc.robot.Subsystems.ReadyforTuning.IntakeSub;
import frc.robot.Subsystems.ReadyforTuning.ShooterSub;
import frc.robot.Subsystems.ReadyforTuning.SwerveSub;
import frc.robot.Subsystems.ReadyforTuning.TurretSub;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
private final ClimberSub climberSub = new ClimberSub();
private final IntakeSub intakeSub = new IntakeSub();
private final ShooterSub shootersub = new ShooterSub();
private final SwerveSub drivebase = new SwerveSub();
private final TurretSub turretSub = new TurretSub();

  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final GenericHID buttonBox = new GenericHID(0);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();


  public RobotContainer() {
    configureBindings();
          // Put the chooser on Shuffleboard
        SmartDashboard.putData("Auto Mode", autoChooser);
        // Set a default auto so it runs even if you don't pick one
        autoChooser.setDefaultOption("MyAuto", new PathPlannerAuto("MyAuto"));
        
  }
SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), //gives inputs to swerve from left joystick for translating
                                                                () -> driverController.getLeftY() * -1,
                                                                () -> driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(3) //SPEED CHANGE
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX, //gives inputs from the right joystick for turning
                                                                                             driverController::getRightY)
                                                           .headingWhile(true);
 
Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle); //allows robot to move at field oriented angle
Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  


//BUTTON BOX LAYOUT
//    +--------------------------------+
//    |  (1)   (3)   (5)   (7)   (9)   |
//    |                                |
//    |  (2)   (4)   (6)   (8)   (10)  |
//    +--------------------------------+

private void configureBindings() { //default 
  
    //XBOX CONTROLLER
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    driverController.a().onTrue(new InstantCommand (drivebase::zeroGyro, drivebase));
    //driverController.a().onTrue(new InstantCommand (drivebase::zeroGyro, drivebase));

    //BUTTON BOX CONTROLS

    //turret Controls
      new JoystickButton(buttonBox, 5).onTrue(new RunCommand(turretSub::setpointRight, turretSub));
      new JoystickButton(buttonBox, 7).onTrue(new RunCommand(turretSub::setpointCenter, turretSub));
      new JoystickButton(buttonBox, 9).onTrue(new RunCommand(turretSub::setpointLeft, turretSub));

    //Shooter controls
      new JoystickButton(buttonBox, 3).onTrue(new RunCommand(shootersub::FlywheelSetRPM, shootersub));
      new JoystickButton(buttonBox, 4).onTrue(new RunCommand(shootersub::ShooterUnjam, shootersub));

    //Climber Controls
      new JoystickButton(buttonBox, 1).onTrue(new InstantCommand(climberSub::ClimberMotorUp, climberSub));
            new JoystickButton(buttonBox, 1).onFalse(new InstantCommand(climberSub::ClimberMotorStop, climberSub));
      new JoystickButton(buttonBox, 2).onTrue(new InstantCommand(climberSub::ClimberMotorDown, climberSub));
            new JoystickButton(buttonBox, 2).onFalse(new InstantCommand(climberSub::ClimberMotorStop, climberSub));
      
  }


  // public Command getAutonomousCommand(String pathName) {
  //   // Create a path following command using AutoBuilder. This will also trigger event markers. //
  //   return drivebase.getAutonomousCommandSub("New Auto");
  // }

    public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}

