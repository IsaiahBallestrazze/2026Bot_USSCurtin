// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ReadyforTuning.TurretSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Turret extends Command {
  /** Creates a new Turret. */
  private final TurretSub turretSub;
  private final double angle;

  public Turret(TurretSub turretSubL, double angleL) {
turretSub = turretSubL;
  angle = angleL;
addRequirements(turretSub);}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Turret Command Initialized with angle: " + angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        SmartDashboard.putNumber("Turret Current Angle", Math.toDegrees(turretSub.getTurretAngle()));
        SmartDashboard.putNumber("Turret Current RPM", turretSub.getTurretRPM());
        turretSub.setTurretAngle(angle);

        

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        System.out.println("Turret Command Ended with angle: " + angle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
