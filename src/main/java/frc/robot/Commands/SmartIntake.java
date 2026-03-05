// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ReadyforTuning.IntakeSub;
import frc.robot.Subsystems.ReadyforTuning.ShooterSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartIntake extends Command {
  /** Creates a new SmartIntake. */
  private final IntakeSub intakeSub;
  private final ShooterSub shooterSub;
  private final GenericHID buttonBox;

  public SmartIntake(IntakeSub l_intakeSub, ShooterSub l_shooterSub, GenericHID l_buttonBox) {
     intakeSub = l_intakeSub;
     shooterSub = l_shooterSub;
     buttonBox = l_buttonBox;

     addRequirements(l_intakeSub, l_shooterSub);
    // Use addRequirements() here to declare subsystem dependencies.
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
  public void end(boolean interrupted) {
    intakeSub.IntakeGroup(0, 0);
    intakeSub.IntakeArmSet(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !buttonBox.getRawButton(3);
  }
}
