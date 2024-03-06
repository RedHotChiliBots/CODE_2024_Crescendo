// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class IntakeStop extends Command {
  private Intake intake = null;
  private Feeder feeder = null;
  /** Creates a new IntakeNote. */
  public IntakeStop(Intake intake, Feeder feeder) {
    this.intake = intake;
    this.feeder = feeder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stopIntake();
    feeder.stopFeeder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
