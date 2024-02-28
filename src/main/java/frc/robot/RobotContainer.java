// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TrapperConstants;
import frc.robot.autos.Autos;
import frc.robot.commands.ClimberLift;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootNote;
import frc.robot.commands.ShooterTilt;
import frc.robot.commands.TrapperLift;
import frc.robot.commands.TrapperTilt;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trapper;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final Chassis chassis = new Chassis();
	private final Autos auton = new Autos(chassis);
	private final Climber climber = new Climber();
	private final Intake intake = new Intake();
	private final Feeder feeder = new Feeder();
	private final Shooter shooter = new Shooter();
	private final Trapper trapper = new Trapper();
	private final Vision vision = new Vision(chassis);

	// The driver's controller
	XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
	XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

	IntakeNote intakeNote = new IntakeNote(intake, feeder, shooter);
	ShootNote shootNote = new ShootNote(feeder, shooter);

	ClimberLift climberLiftTop = new ClimberLift(climber, ClimberConstants.kMaxClimbPos);
	ClimberLift climberLiftBot = new ClimberLift(climber, ClimberConstants.kMinClimbPos);
	ShooterTilt shooterTiltTop = new ShooterTilt(shooter, ShooterConstants.kMaxTiltPos);
	ShooterTilt shooterTiltBot = new ShooterTilt(shooter, ShooterConstants.kMinTiltPos);
	TrapperLift trapperLiftTop = new TrapperLift(trapper, TrapperConstants.kMaxLiftLen);
	TrapperLift trapperLiftBot = new TrapperLift(trapper, TrapperConstants.kMinLiftLen);
	TrapperTilt trapperTiltTop = new TrapperTilt(trapper, TrapperConstants.kMaxTiltDeg);
	TrapperTilt trapperTiltBot = new TrapperTilt(trapper, TrapperConstants.kMinTiltDeg);

	private final ShuffleboardTab chassisTab = Shuffleboard.getTab("Chassis");
	private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
	private final ShuffleboardTab feederTab = Shuffleboard.getTab("Feeder");
	private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
	private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
	private final ShuffleboardTab trapperTab = Shuffleboard.getTab("Trapper");

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		chassisTab.add("Chassis", chassis);
		intakeTab.add("Intake", intake);
		feederTab.add("Feeder", feeder);
		shooterTab.add("Shooter", shooter);
		climberTab.add("Climber", climber);
		trapperTab.add("Trapper", trapper);

		intakeTab.add("IntakeNote", intakeNote);
		shooterTab.add("ShootNote", shootNote);

		shooterTab.add("ShooterTiltTop", shooterTiltTop);
		shooterTab.add("ShooterTiltBot", shooterTiltBot);
		trapperTab.add("TrapperLiftTop", trapperLiftTop);
		trapperTab.add("TrapperLiftBot", trapperLiftBot);
		climberTab.add("ClimberLiftTop", climberLiftTop);
		climberTab.add("ClimberLiftBot", climberLiftBot);
		trapperTab.add("TrapperTiltTop", trapperTiltTop);
		trapperTab.add("TrapperTiltBot", trapperTiltBot);

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		chassis.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				new RunCommand(
						() -> chassis.drive(
								-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
								-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
								-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
								true, false),
						chassis));
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
		new JoystickButton(m_driverController, Button.kX.value)
				.whileTrue(new RunCommand(
						() -> chassis.setX(),
						chassis));

		new JoystickButton(m_driverController, Button.kB.value).debounce(1)
				.onTrue(new RunCommand(
						() -> vision.setInsertOffset(true),
						vision));

		new JoystickButton(m_driverController, Button.kA.value)
				.whileTrue(new RunCommand(
						() -> vision.trackAprilTag(),
						vision));

		new JoystickButton(m_driverController, Button.kY.value).debounce(1)
				.onTrue(new RunCommand(
						() -> chassis.zeroYaw(),
						chassis));

		new JoystickButton(m_operatorController, Button.kX.value).debounce(1)
				.onTrue(intakeNote);

		new JoystickButton(m_operatorController, Button.kY.value).debounce(1)
				.onTrue(shootNote);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return auton.getChooser().getSelected();
	}
}
