package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.Library;

import org.junit.jupiter.api.BeforeAll;

public class LibraryTest {

	private final Library lib = new Library();

	@BeforeAll
	public static void setup() {

	}

	@Test
	public void testInt() {
		assertEquals(5, lib.Clip(5, 10, 5));
		assertEquals(9, lib.Clip(9, 10, 5));
		assertEquals(5, lib.Clip(4, 10, 5));
		assertEquals(10, lib.Clip(10, 10, 5));
		assertEquals(5, lib.Clip(5, 10, 5));
		assertEquals(10, lib.Clip(11, 10, 5));
		assertEquals(5, lib.Clip(4, 10, 5));
	}

	@Test
	public void testDouble() {
		assertEquals(5.0, lib.Clip(5.0, 10.0, 5.0));
		assertEquals(9.9, lib.Clip(9.9, 10.0, 5.0));
		assertEquals(5.0, lib.Clip(4.9, 10.0, 5.0));
		assertEquals(10.0, lib.Clip(10.0, 10.0, 5.0));
		assertEquals(5.0, lib.Clip(5.0, 10.0, 5.0));
		assertEquals(10.0, lib.Clip(10.1, 10.0, 5.0));
		assertEquals(5.1, lib.Clip(5.1, 10.0, 5.0));
	}
}
