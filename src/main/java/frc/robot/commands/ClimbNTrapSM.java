// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TrapperConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Trapper;
import frc.robot.subsystems.Trapper.CLAW;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbNTrapSM extends SequentialCommandGroup {
  /** Creates a new TrappNClimb. */
  public ClimbNTrapSM(Trapper trapper, Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new JustClimb(climber, -0.40, 4.0),
        new WaitCommand(1.0), //1
        new ParallelCommandGroup(
            new RunCommand(() -> trapper.holdLift(18.0)),
            new RunCommand(() -> trapper.holdTilt(TrapperConstants.kTrapScoreTiltDeg))),
        new WaitCommand(0.5), //0.5
        new RunCommand(() -> trapper.holdClaw(CLAW.OPEN)),
        new WaitCommand(0.5), //1
        new RunCommand(() -> trapper.holdLift(0.0), trapper));
  }
}
