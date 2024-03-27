// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;

public class AutoTrackAprilTag extends Command {
  Chassis chassis;
  Vision vision;
  int targetId;

  /** Creates a new AutonTrackAprilTag. */
  public AutoTrackAprilTag(Chassis chassis, Vision vision) {
    this.chassis = chassis;
    this.vision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (vision.getHasTargets()) {

    //   double[] spd = vision.trackAprilTag();

    //   // Use our forward/turn speeds to control the drivetrain
    //   chassis.drive(spd[0], spd[1], 0.0, true, false);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;  //vision.atDistTarget();
  }
}
