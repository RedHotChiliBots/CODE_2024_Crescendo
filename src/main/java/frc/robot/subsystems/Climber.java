// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");

  public Climber() {
    System.out.println("+++++ Starting Climber Constructor +++++");
    System.out.println("----- Ending Climber Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
