// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Trapper;
import frc.robot.subsystems.Trapper.CLAW;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbNTrap extends SequentialCommandGroup {
  /** Creates a new TrappNClimb. */
  public ClimbNTrap(Trapper trapper, Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new JustClimb(climber, -0.40, 4.0),
        new WaitCommand(1.0),
        new ParallelCommandGroup(
            new RunCommand(() -> trapper.setLiftSP(18.0)),
            new RunCommand(() -> trapper.setTiltSP(120.0))),
        new WaitCommand(0.5),
        new TrapperClaw(trapper, CLAW.OPEN),
        new WaitCommand(1.0),
        new RunCommand(() -> trapper.setLiftSP(0.0)));
  }
}
