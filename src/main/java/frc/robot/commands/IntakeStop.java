// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeStop extends Command {
  private Intake intake = null;
  private Feeder feeder = null;
  private Shooter shooter = null;

  /** Creates a new IntakeNote. */
  public IntakeStop(Intake intake, Feeder feeder, Shooter shooter) {
    this.intake = intake;
    this.feeder = feeder;
    this.shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, feeder, shooter);
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
