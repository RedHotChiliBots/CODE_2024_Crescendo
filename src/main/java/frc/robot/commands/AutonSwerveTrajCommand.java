// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.Constants.ChassisConstants;

public class AutonSwerveTrajCommand extends SwerveControllerCommand {
  /** Creates a new DriveTrajectory. */
//  Chassis chassis;

  public AutonSwerveTrajCommand(Chassis chassis, Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.

    super(
				trajectory,
				chassis::getPose,
				ChassisConstants.kDriveKinematics,
				chassis.holonomicController,
				chassis::setModuleStates,
				chassis);

//    this.chassis = chassis;
    addRequirements(chassis);
  }
}