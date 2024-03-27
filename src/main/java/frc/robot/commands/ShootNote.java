// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {
  Intake intake = null;
  Feeder feeder = null;
  Shooter shooter = null;
  Timer timer = new Timer();
  boolean oneTime = false;

  /** Creates a new ShootNote. */
  public ShootNote(Intake intake, Feeder feeder, Shooter shooter) {
    this.intake = intake;
    this.feeder = feeder;
    this.shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    // shooter.setVelocity(ShooterConstants.kShootVelocity);
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oneTime = false;
    shooter.holdVelocity(ShooterConstants.kShootVelocity);
    shooter.holdTilt(ShooterConstants.kMaxTiltPos);
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(0.6) && !oneTime) {
      feeder.holdVelocity(FeederConstants.kFeederVelocity);
      intake.holdVelocity(IntakeConstants.kIntakeVelocity);
      oneTime = true;
      timer.reset();
      timer.restart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    feeder.stopFeeder();
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return oneTime && timer.hasElapsed(2.0);
  }
}
