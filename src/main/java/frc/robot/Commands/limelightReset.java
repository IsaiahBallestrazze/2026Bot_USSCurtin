// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ReadyforTuning.SwerveSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class limelightReset extends Command {
  /** Creates a new limelightReset. */
      private final SwerveSub swerveSub;
      private final double[] xPositionData = new double[9];
      private final double[] yPositionData = new double[9];

      public limelightReset(SwerveSub d_swerveSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveSub = d_swerveSub;
    addRequirements(swerveSub);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSub.updateVisionOdometryReal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

