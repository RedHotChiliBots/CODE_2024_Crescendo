// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.Random;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class VisionUtils {

	private static Random rand = new Random();

	public static Pose2d BumpPose(Pose2d pose) {
		// System.out.printf("Random: %7.5fd\n", rand.nextDouble() * 4 -2);

		System.out.println("Pose: " + pose);

		Transform2d trf = new Transform2d(
				new Translation2d(rand.nextDouble() * 4 - 2, rand.nextDouble() * 4 - 2),
				new Rotation2d(rand.nextDouble() * 2 * Math.PI));

		System.out.println("TRF: " + trf);

		return pose.plus(trf);
	}
}
