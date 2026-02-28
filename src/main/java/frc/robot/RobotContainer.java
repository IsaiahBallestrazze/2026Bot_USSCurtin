// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.SmartShoot;
import frc.robot.Commands.Turret;
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
//Turret trackTarget = new Turret(turretSub, 0);

  private final CommandXboxController driverController = new CommandXboxController(1);
    private final GenericHID buttonBox = new GenericHID(0);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

//TARGET POSITION ON FIELD
private double bluealliancetargetX = 6.23; //X position of target
private double bluealliancetargetY = 4.02; //Y position of target




  public RobotContainer() {
    configureBindings();
          // Put the chooser on Shuffleboard
        SmartDashboard.putData("Auto Mode", autoChooser);
        // Set a default auto so it runs even if you don't pick one
        autoChooser.setDefaultOption("MyAuto", new PathPlannerAuto("MyAuto"));
        
  }
SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), //gives inputs to swerve from left joystick for translating
                                                                () -> driverController.getLeftY() * -1, //inverts controller
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
    driverController.b().whileTrue(new RunCommand (()-> turretSub.setFlywheelRPM(turretSub.calculateSetpointRPM(turretSub.calculateDistance(turretSub.getFieldPositionX(), turretSub.getFieldPositionY(), bluealliancetargetX, bluealliancetargetY))), turretSub));
    driverController.b().onFalse(new RunCommand (()-> turretSub.setFlywheelSpeed(0), turretSub));
    

    // driverController.b().onTrue(new RunCommand(() -> System.out.println("ontrue")));
    // driverController.b().whileTrue(new RunCommand(() -> System.out.println("whileTrue")));
    // driverController.b().toggleOnTrue(new RunCommand(() -> System.out.println("toggleOnTrue")));

    //driverController.a().onTrue(new InstantCommand (drivebase::zeroGyro, drivebase));

    //BUTTON BOX CONTROLS

    //turret Controls


    //Shooter controls
      // new JoystickButton(buttonBox, 3).onTrue(new RunCommand(shootersub::FlywheelSetRPM, shootersub));
      // new JoystickButton(buttonBox, 4).onTrue(new RunCommand(shootersub::ShooterUnjam, shootersub));

    //Climber Controls
      // new JoystickButton(buttonBox, 1).onTrue(new InstantCommand(climberSub::ClimberMotorUp, climberSub));
      //       new JoystickButton(buttonBox, 1).onFalse(new InstantCommand(climberSub::ClimberMotorStop, climberSub));
      // new JoystickButton(buttonBox, 2).onTrue(new InstantCommand(climberSub::ClimberMotorDown, climberSub));
      //       new JoystickButton(buttonBox, 2).onFalse(new InstantCommand(climberSub::ClimberMotorStop, climberSub));
      


     new JoystickButton(buttonBox, 1).onTrue(new RunCommand(() -> intakeSub.IntakeArmSet(-1), intakeSub)); //UP
        new JoystickButton(buttonBox, 1).onFalse(new RunCommand(() -> intakeSub.IntakeArmSet(0), intakeSub));

      new JoystickButton(buttonBox, 2).onTrue(new RunCommand(() -> intakeSub.IntakeArmSet(.5), intakeSub)); //DOWN
            new JoystickButton(buttonBox, 2).onFalse(new RunCommand(() -> intakeSub.IntakeArmSet(0), intakeSub));

      new JoystickButton(buttonBox, 3).onTrue(new RunCommand(() -> intakeSub.IntakeGroup(1,-.5), intakeSub));
            new JoystickButton(buttonBox, 3).onFalse(new RunCommand(() -> intakeSub.IntakeGroup(0,0), intakeSub));

            ///////
     new JoystickButton(buttonBox, 4).onTrue(new RunCommand(() -> shootersub.ShooterGroup(.7,.5), shootersub));
            new JoystickButton(buttonBox, 4).onFalse(new RunCommand(() -> shootersub.ShooterGroup(0,0), shootersub));

     new JoystickButton(buttonBox, 6).onTrue(new RunCommand(() -> shootersub.ShooterGroup(-.7,-.5), shootersub));
        new JoystickButton(buttonBox, 6).onTrue(new RunCommand(() -> turretSub.setFlywheelSpeed(-.5), turretSub));

      new JoystickButton(buttonBox, 6).onFalse(new RunCommand(() -> shootersub.ShooterGroup(0,0), shootersub));
          new JoystickButton(buttonBox, 6).onFalse(new RunCommand(() -> turretSub.setFlywheelSpeed(0), turretSub));

            ////////////////////////
     new JoystickButton(buttonBox, 5).onTrue(new RunCommand(() -> turretSub.setFlywheelRPM(4500), turretSub));
          new JoystickButton(buttonBox, 5).onTrue(new RunCommand(() -> intakeSub.AgitatorSet(.5), intakeSub));
                    new JoystickButton(buttonBox, 5).onTrue(new RunCommand(() -> shootersub.ShooterGroup(1,1), shootersub));
              // new JoystickButton(buttonBox, 5).onTrue(new RunCommand(() -> turretSub.setFlywheelRPM(400), turretSub));

     
    //new JoystickButton(buttonBox, 5).onTrue(new SmartShoot(shootersub, turretSub, intakeSub, buttonBox, bluealliancetargetX, bluealliancetargetY));
    new JoystickButton(buttonBox, 5).onFalse(new RunCommand(() -> turretSub.setFlywheelSpeed(0), turretSub));
        new JoystickButton(buttonBox, 5).onFalse(new RunCommand(() -> intakeSub.AgitatorSet(0), intakeSub));
                new JoystickButton(buttonBox, 5).onFalse(new RunCommand(() -> shootersub.ShooterGroup(0,0), shootersub));


    new JoystickButton(buttonBox, 7).whileTrue(new RunCommand(() -> drivebase.updateVisionOdometryReal(), turretSub));

      // //fear this single line of code
      new JoystickButton(buttonBox, 8).toggleOnTrue(new RunCommand(() -> turretSub.setTurretspeedWithlimits(
        turretSub.CalculateRotationSpeed(turretSub.CalculateTargetAngle(
                                                                        turretSub.getFieldPositionX(), 
                                                                        turretSub.getFieldPositionY(), 
                                                                        bluealliancetargetX, 
                                                                        bluealliancetargetY),
        turretSub.getTurretAngle(), 
        turretSub.getTurretPID(), 
        Math.toRadians(drivebase.getAnglesInverted()))))); //converts the gyro to radians for math


      //new JoystickButton(buttonBox, 1).toggleOnTrue(new RunCommand(() -> climberSub.getClimberPosition(), climberSub));
      //new JoystickButton(buttonBox, 2).toggleOnTrue(new RunCommand(() -> intakeSub.getIntakeArmPosition(), intakeSub));
      new JoystickButton(buttonBox, 3).onTrue(new RunCommand(() -> turretSub.getTurretAngle(), turretSub));



















            new JoystickButton(buttonBox, 9).onTrue(new RunCommand(() -> climberSub.ClimberSetSpeed(-1), climberSub)); //negative brings climber down
            new JoystickButton(buttonBox, 9).onFalse(new RunCommand(() -> climberSub.ClimberSetSpeed(0), climberSub));

            new JoystickButton(buttonBox, 10).onTrue(new RunCommand(() -> climberSub.ClimberSetSpeed(1), climberSub)); //negative brings climber down
            new JoystickButton(buttonBox, 10).onFalse(new RunCommand(() -> climberSub.ClimberSetSpeed(0), climberSub));

            // new JoystickButton(buttonBox, 5).onTrue(new RunCommand(() -> climberSub.ClimberMotorDown(), climberSub));
            //             new JoystickButton(buttonBox, 5).onFalse(new RunCommand(() -> climberSub.ClimberSetSpeed(-.5), climberSub));


      //   new JoystickButton(buttonBox, 1).onTrue(new RunCommand(() -> turretSub.setTurretspeed(.5), turretSub));
      //   new JoystickButton(buttonBox, 1).onFalse(new RunCommand(() -> turretSub.setTurretspeed(0), turretSub));

      //   //         new JoystickButton(buttonBox, 2).onTrue(new RunCommand(() -> turretSub.setTurretspeed(1), turretSub));
      //   // new JoystickButton(buttonBox, 2).onFalse(new RunCommand(() -> turretSub.setTurretspeed(0), turretSub));

      // new JoystickButton(buttonBox, 3).onTrue(new RunCommand(() -> turretSub.setTurretAngle((Math.toRadians(45)))));
      // new JoystickButton(buttonBox, 4).onTrue(new RunCommand(() -> turretSub.setTurretAngle(Math.toRadians(135))));


      // //new JoystickButton(buttonBox, 10).onTrue(new RunCommand(() -> turretSub.SetFieldPosition(0, 0, 0), turretSub));
      // //new JoystickButton(buttonBox, 10).onTrue(new RunCommand(() -> System.out.println(drivebase.getGyroRaw()), turretSub));
      //   new JoystickButton(buttonBox, 10).whileTrue(new RunCommand(() -> drivebase.updateVisionOdometryReal(), turretSub));

      // new JoystickButton(buttonBox, 9).onTrue(new RunCommand(() -> turretSub.printoutLimelightData(), turretSub));

      // new JoystickButton(buttonBox, 5).toggleOnTrue(new RunCommand(() -> turretSub.getTurretPosition(), turretSub));
      // new JoystickButton(buttonBox, 6).onTrue(new RunCommand(() -> turretSub.resetTurretEncoder(), turretSub));




      // //fear this single line of code
      // new JoystickButton(buttonBox, 8).toggleOnTrue(new RunCommand(() -> turretSub.setTurretspeedWithlimits(
      //   turretSub.CalculateRotationSpeed(turretSub.CalculateTargetAngle(
      //                                                                   turretSub.getFieldPositionX(), 
      //                                                                   turretSub.getFieldPositionY(), 
      //                                                                   bluealliancetargetX, 
      //                                                                   bluealliancetargetY),
      //   turretSub.getTurretAngle(), 
      //   turretSub.getTurretPID(), 
      //   Math.toRadians(drivebase.getAnglesInverted()))))); //converts the gyro to radians for math





  }


  // public Command getAutonomousCommand(String pathName) {
  //   // Create a path following command using AutoBuilder. This will also trigger event markers. //
  //   return drivebase.getAutonomousCommandSub("New Auto");
  // }

    public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}

