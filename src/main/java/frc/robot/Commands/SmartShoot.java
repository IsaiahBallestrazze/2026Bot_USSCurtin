// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.ReadyforTuning.IntakeSub;
import frc.robot.Subsystems.ReadyforTuning.ShooterSub;
import frc.robot.Subsystems.ReadyforTuning.TurretSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartShoot extends Command {
  /** Creates a new SmartShoot. */

ShooterSub shooterSub;
TurretSub turretSub;
IntakeSub intakeSub;
GenericHID buttonBox;
double targetRPM;

// private final double turretConstant = 500; //turret constant to convert distance to RPM
// private final double turretOffset = 300;

  public SmartShoot(ShooterSub l_shooterSub, TurretSub l_turretSub, IntakeSub l_intakeSub, GenericHID l_buttonBox) {
    shooterSub = l_shooterSub;
    turretSub = l_turretSub;
    intakeSub = l_intakeSub;
    buttonBox = l_buttonBox;

    addRequirements(shooterSub, turretSub, intakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
          intakeSub.IntakeArmSet(-1); //up is negative probably
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // targetRPM = ((turretSub.calculateDistance(turretSub.getFieldPositionX(), turretSub.getFieldPositionY(), targetx, targety)) * turretConstant) + turretOffset; //turret constant LINEAR BOO
    targetRPM = turretSub.calculateSetpointRPM(turretSub.calculateDistance(
                                                                            turretSub.getFieldPositionX(),
                                                                            turretSub.getFieldPositionY()
                                                                            ));
    turretSub.setFlywheelRPM(targetRPM);
    System.out.println("SHOOTING");
    // System.out.println(turretSub.getFlywheelRPM());

    if((turretSub.getFlywheelRPM() > (targetRPM - 20)) && (turretSub.getFlywheelRPM() < (targetRPM + 20))){ //if flywheel is at target RPM, start feeding balls
      shooterSub.ShooterGroup(1, 1);
      intakeSub.AgitatorSet(-.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //if((turretSub.getFlywheelRPM() > (targetRPM - 100)) && (turretSub.getFlywheelRPM() < (targetRPM + 100))){      shooterSub.ShooterGroup(0, 0);
      intakeSub.AgitatorSet(0);
      intakeSub.IntakeArmSet(0);
      shooterSub.mecanumSet(0);
      shooterSub.greenWheelSet(0);
    //}

      
      turretSub.setFlywheelSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !buttonBox.getRawButton(5); //buttonBox.getRawButtonReleased(5)

  }
}
