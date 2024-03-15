// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trapper;
import frc.robot.subsystems.Trapper.CLAW;

public class ShootPosNote extends Command {
  Trapper trapper = null;
  Intake intake = null;
  Feeder feeder = null;
  Shooter shooter = null;
  Timer timer = new Timer();
  boolean finished = false;
  double pos = 0.0;
  int state = 0;

  /** Creates a new ShootNote. */
  public ShootPosNote(Trapper trapper, Intake intake, Feeder feeder, Shooter shooter) {
    this.trapper = trapper;
    this.intake = intake;
    this.feeder = feeder;
    this.shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    // shooter.setVelocity(ShooterConsktants.kShootVelocity);
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    state = 0;
    feeder.holdVelocity(FeederConstants.kFeederVelocity);
    shooter.holdVelocity(ShooterConstants.kMoveVelocity);
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {

      case 0:
        if (!intake.isNoteDetected()) {
          feeder.stopFeeder();
          shooter.holdPosition(shooter.getPosition());
          state++;
        }
        break;

      case 1:
        trapper.holdClaw(CLAW.CLOSE);
        finished = true;
        break;

      default:
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
