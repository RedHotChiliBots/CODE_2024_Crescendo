// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapSetup extends SequentialCommandGroup {
  /** Creates a new TrappNClimb. */
  public TrapSetup(Trapper trapper, Climber climber, Intake intake, Feeder feeder, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShootPosNote(trapper, intake, feeder, shooter),
        new JustClimb(climber, 0.25, ClimberConstants.kMaxClimbPos));
        // new ParallelCommandGroup(
        //     // new RunCommand(() -> shooter.setTiltSP(ShooterConstants.kMaxTiltPos)),
        //     new JustClimb(climber, 0.25, 18.0),
        //     new RunCommand(() -> trapper.setTiltSP(TrapperConstants.kSetupTiltDeg))));
  }
}
