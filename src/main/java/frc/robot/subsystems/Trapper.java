// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Trapper extends SubsystemBase {
  /** Creates a new Trapper. */
    private final ShuffleboardTab trapperTab = Shuffleboard.getTab("Trapper");
  public Trapper() {
    System.out.println("+++++ Starting Trapper Constructor +++++");
    System.out.println("----- Ending Trapper Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
