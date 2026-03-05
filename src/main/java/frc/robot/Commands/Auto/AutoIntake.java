// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ReadyforTuning.IntakeSub;
import frc.robot.Subsystems.ReadyforTuning.ShooterSub;
import frc.robot.Subsystems.ReadyforTuning.SwerveSub;
import frc.robot.Subsystems.ReadyforTuning.TurretSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoIntake extends Command {
  /** Creates a new AutoIntake. */

ShooterSub shooterSub;
TurretSub turretSub;
IntakeSub intakeSub;
double targetRPM;
SwerveSub drivebase;

  public AutoIntake(ShooterSub l_shooterSub, TurretSub l_turretSub, IntakeSub l_intakeSub, SwerveSub l_driveBase) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSub = l_shooterSub;
    turretSub = l_turretSub;
    intakeSub = l_intakeSub;
    drivebase = l_driveBase;
    addRequirements(l_shooterSub, l_turretSub, l_intakeSub, l_driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("INTAKING");
    intakeSub.IntakeGroup(1, -.5);
    intakeSub.IntakeArmSet(1); // DOWN
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSub.getIntakeWheelPosition() > 100; //if doesent work may need to invert return value
  }
}
