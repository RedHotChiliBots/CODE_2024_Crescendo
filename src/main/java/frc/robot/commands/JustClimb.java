// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;

public class JustClimb extends Command {
  /** Creates a new TrapClawOpen. */

  private Climber climber = null;
  private Chassis chassis = null;
  private double pos = 0.0;
  private Timer timer = new Timer();

  public JustClimb(Climber climber, Chassis chassis, double pos) {
    this.climber = climber;
    this.pos = pos;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.setChannelOff();

    if (pos < 0.0) {
      climber.climb(pos);
    }
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(0.1)) {
      climber.climb(pos);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setChannelOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
