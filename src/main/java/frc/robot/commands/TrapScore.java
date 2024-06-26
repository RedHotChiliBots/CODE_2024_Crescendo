// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TrapperConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Trapper;
import frc.robot.subsystems.Trapper.CLAW;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapScore extends SequentialCommandGroup {
  /** Creates a new TrappNClimb. */
  public TrapScore(Trapper trapper, Climber climber, Chassis chassis) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> chassis.setCalcPitch(true)),
        new JustClimb(climber, -0.40, 0.0),
        new WaitCommand(1.0), //3
        // new ParallelCommandGroup(
        new InstantCommand(() -> trapper.holdTilt(TrapperConstants.kTrapScoreTiltDeg), trapper),
        new InstantCommand(() -> trapper.holdLift(TrapperConstants.kTrapScoreLiftLen), trapper),
        // ),
        new WaitCommand(1.5), //2 //2.5
        new InstantCommand(() -> trapper.holdClaw(CLAW.TRAP), trapper),
        new WaitCommand(0.7), //.3 //2
        new InstantCommand(() -> trapper.holdLift(0.0), trapper, trapper));
  }
}
