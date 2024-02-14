package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.junit.jupiter.api.BeforeAll;    
    
public class VisionUtilsTest {

    @BeforeAll
    public static void setup() {

    }
        
    @Test
	public void testbumpPose() {
		//assertEquals(5.5, VisionUtils.BumpPose(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0))));
		System.out.print("Result: " + VisionUtils.BumpPose(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0))));
	}
}
    