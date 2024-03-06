// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trapper;

public class TrapperLift extends Command {
  /** Creates a new TrapClawOpen. */

  private Trapper trapper = null;
  private double len = 0.0;

  public TrapperLift(Trapper trapper, double len) {
    this.trapper = trapper;
    this.len = len;

    // Use addRequirements() here to declare subsystem dependencies.
    // Tilt should not interrup Shooter.  No dependency on Shooter.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trapper.holdLift(len);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
