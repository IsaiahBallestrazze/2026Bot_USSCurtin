// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ReadyforTuning.ClimberSub;
import frc.robot.Subsystems.ReadyforTuning.IntakeSub;
import frc.robot.Subsystems.ReadyforTuning.ShooterSub;
import frc.robot.Subsystems.ReadyforTuning.SwerveSub;
import frc.robot.Subsystems.ReadyforTuning.TurretSub;



//AUTO COMMAND IS THE SAME EXCEPT IT WAITS UNTIL ENDOER OF FLYWHEEL REACHES A CERTAIN DISTANCE BEFORE STOPPING
public class AutoClimbDown extends Command {
ClimberSub climberSub;
SwerveSub drivebase;

// private final double turretConstant = 500; //turret constant to convert distance to RPM
// private final double turretOffset = 300;

  public AutoClimbDown(ClimberSub l_climberSub, SwerveSub l_driveBase) {
    climberSub = l_climberSub;
    drivebase = l_driveBase;

    addRequirements(climberSub, drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    climberSub.ClimberSetSpeed(-1);
    //FLYWHEEL
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSub.ClimberSetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climberSub.getClimberPosition() < 5; //if doesent work may need to invert return value
  }
}
