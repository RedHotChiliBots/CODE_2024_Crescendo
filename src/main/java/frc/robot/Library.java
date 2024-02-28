// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Library {
	
	// Math functions
	public double Clip(double value, double max, double min) {
		return Math.min(Math.max(value, min), max);
	}

	public int Clip(int value, int max, int min) {
		return Math.min(Math.max(value, min), max);
	}
}
