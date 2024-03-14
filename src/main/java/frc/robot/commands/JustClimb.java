// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class JustClimb extends Command {
  /** Creates a new TrapClawOpen. */

  private Climber climber = null;
  private double spd = 0.0;
  private double pos = 0.0;
  private Timer timer = new Timer();

  public JustClimb(Climber climber, double spd, double pos) {
    this.climber = climber;
    this.spd = spd;
    this.pos = pos;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // positive is down
  // negative is up
  // on down, disengage(on) solenoid and go up(+) briefly then go down(-), finish
  // engage solenoid(off)
  // on up, disengage(on) solenoid and go up(+), finish engage solenoid(off)

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (pos >= 0.0)
      climber.setClimbSP(pos);
    climber.setBrakeOn();

    // positive spd goes UP, negative spd goes DN
    if (pos >= 0.0) { // if commanding set point
      if (climber.getLeftPosition() < pos) {  // Going UP
        spd = Math.abs(spd);  // Set spd to positive
        climber.climb(-spd);  // Reverse briefly to release brake
      } else {                                // Going DN
        spd = -Math.abs(spd); // Set spd to negative
        climber.climb(spd);   // No need to reverse
      }
    } else if (spd > 0.0) { // if not set point, tap UP
      climber.climb(-spd);  // reverse briefly to release brake
    }
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(0.075)) {  // after brief time
      climber.climb(spd);                   // go UP/DN as intended
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setBrakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pos >= 0.0 && climber.atClimberSP();
  }
}
