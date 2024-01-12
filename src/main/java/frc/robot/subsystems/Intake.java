// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
    private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
  public Intake() {
    System.out.println("+++++ Starting Intake Constructor +++++");
    System.out.println("----- Ending Intake Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
