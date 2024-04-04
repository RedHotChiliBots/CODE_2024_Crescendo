// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.Autos;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonSpeakerAmp extends SequentialCommandGroup {
  /** Creates a new AutonSpeakerAmp. */
  public AutonSpeakerAmp(Chassis chassis, Autos autos, Vision vision, Intake intake, Feeder feeder, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootNote(intake, feeder, shooter),
      new AutonSwerveTrajCommand(chassis, autos.note1Trajectory)//,
      // new AutonTrackAprilTag(chassis, vision, 7),
      // new AutonSwerveTrajCommand(chassis, autos.note2Trajectory),
      // new AutonTrackAprilTag(chassis, vision, 6),
      // new AutonSwerveTrajCommand(chassis, autos.note3Trajectory),
      // new AutonTrackAprilTag(chassis, vision, 6)
    );
  }
}
