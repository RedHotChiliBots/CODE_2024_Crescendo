// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberHoldPos extends Command {
  /** Creates a new TrapClawOpen. */

  private Climber climber = null;
  private double pos = 0.0;

  public ClimberHoldPos(Climber climber, double pos) {
    this.climber = climber;
    this.pos = pos;

    // Use addRequirements() here to declare subsystem dependencies.
    // Claw should not interrup Trap. No dependency on Trap.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.holdClimbPos(pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
