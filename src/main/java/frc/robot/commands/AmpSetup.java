// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpSetup extends SequentialCommandGroup {
  /** Creates a new TrappNClimb. */
  public AmpSetup(Trapper trapper, Intake intake, Feeder feeder, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShootPosNote(trapper, intake, feeder, shooter));
        // new WaitCommand(1.0),
        // new InstantCommand(() -> trapper.holdTilt(TrapperConstants.kAmpScoreTiltDeg), trapper),
        // new InstantCommand(() -> trapper.holdLift(TrapperConstants.kAmpScoreLiftLen), trapper));
    // new WaitCommand(2.5),
    // new InstantCommand(() -> trapper.holdClaw(CLAW.OPEN), trapper),
    // new WaitCommand(2.0),
    // new InstantCommand(() -> trapper.holdLift(0.0), trapper, trapper));
  }
}
