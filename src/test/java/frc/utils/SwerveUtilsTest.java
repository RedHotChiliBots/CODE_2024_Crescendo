package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;    
    
public class SwerveUtilsTest {

    @BeforeAll
    public static void setup() {

    }
        
    @Test
	public void testStepTowards() {
		assertEquals(5.0, SwerveUtils.StepTowards(5.0, 5.0, 1.0));
		assertEquals(4.0, SwerveUtils.StepTowards(5.0, 4.0, 1.0));
		assertEquals(5.0, SwerveUtils.StepTowards(4.0, 5.0, 1.0));
	}
}
    