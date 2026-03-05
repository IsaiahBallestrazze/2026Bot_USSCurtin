// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ReadyforTuning.IntakeSub;
import frc.robot.Subsystems.ReadyforTuning.ShooterSub;
import frc.robot.Subsystems.ReadyforTuning.SwerveSub;
import frc.robot.Subsystems.ReadyforTuning.TurretSub;



//AUTO COMMAND IS THE SAME EXCEPT IT WAITS UNTIL ENDOER OF FLYWHEEL REACHES A CERTAIN DISTANCE BEFORE STOPPING
public class AutoShoot extends Command {
ShooterSub shooterSub;
TurretSub turretSub;
IntakeSub intakeSub;
double targetRPM;
SwerveSub drivebase;

// private final double turretConstant = 500; //turret constant to convert distance to RPM
// private final double turretOffset = 300;

  public AutoShoot(ShooterSub l_shooterSub, TurretSub l_turretSub, IntakeSub l_intakeSub, SwerveSub l_driveBase) {
    shooterSub = l_shooterSub;
    turretSub = l_turretSub;
    intakeSub = l_intakeSub;
    drivebase = l_driveBase;

    addRequirements(shooterSub, turretSub, intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretSub.resetFlywheelposition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    //FLYWHEEL
  targetRPM = turretSub.calculateSetpointRPM(turretSub.calculateDistance(
                                                                            turretSub.getFieldPositionX(),
                                                                            turretSub.getFieldPositionY()
                                                                            ));
    turretSub.setFlywheelRPM(targetRPM);
    System.out.println("AUTOSHOOTING");
    System.out.println(turretSub.getFlywheelPosition());
    // System.out.println(turretSub.getFlywheelRPM());

    if((turretSub.getFlywheelRPM() > (targetRPM - 20)) && (turretSub.getFlywheelRPM() < (targetRPM + 20))){ //if flywheel is at target RPM, start feeding balls
      shooterSub.ShooterGroup(1, 1);
      intakeSub.AgitatorSet(-.5);
      intakeSub.IntakeArmSet(-1); //up is negative probably
    }


    //TURRET
    turretSub.setTurretspeedWithlimits(
        turretSub.CalculateRotationSpeed(turretSub.CalculateTargetAngle(
                                                                        turretSub.getFieldPositionX(),
                                                                        turretSub.getFieldPositionY()),
                                          turretSub.getTurretAngle(),
                                          turretSub.getTurretPID(),
                                          Math.toRadians(drivebase.getAnglesInverted())));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

      // if(turretSub.getFlywheelRPM() > (targetRPM - 10)){
      shooterSub.ShooterGroup(0, 0);
      intakeSub.AgitatorSet(0);
      intakeSub.IntakeArmSet(0);
      //}

      
      turretSub.setFlywheelSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turretSub.getFlywheelPosition() > 150; //if doesent work may need to invert return value
  }
}
