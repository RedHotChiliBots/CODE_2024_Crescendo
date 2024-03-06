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
import frc.robot.commands.ClimberStop;
//import frc.robot.commands.AutonChassisDrive;
import frc.robot.commands.ClimberLift;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.JustClimb;
import frc.robot.commands.ShootNote;
import frc.robot.commands.ShooterTilt;
import frc.robot.commands.ShooterTiltStick;
import frc.robot.commands.TrapperLift;
import frc.robot.commands.TrapperClaw;
import frc.robot.commands.TrapperTilt;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Trapper.CLAW;
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
	private final Climber climber = new Climber();
	private final Intake intake = new Intake();
	private final Feeder feeder = new Feeder();
	private final Shooter shooter = new Shooter();
	private final Trapper trapper = new Trapper();
	private final Autos auton = new Autos(chassis, intake, feeder, shooter);
	private final Vision vision = new Vision(chassis);

	// The driver's controller
	XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
	XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

	// AutonChassisDrive autonChassisDrive = new AutonChassisDrive(chassis, 3.0);

	IntakeStop intakeStop = new IntakeStop(intake, feeder);
	IntakeNote intakeNote = new IntakeNote(intake, feeder, shooter);
	ShootNote shootNote = new ShootNote(intake, feeder, shooter);

	ClimberLift climberLiftTop = new ClimberLift(climber, ClimberConstants.kMaxClimbPos);
	ClimberLift climberLiftMid = new ClimberLift(climber,
			((ClimberConstants.kMaxClimbPos + ClimberConstants.kMinClimbPos) / 2.0) - 1.5);
	ClimberLift climberLiftBot = new ClimberLift(climber, ClimberConstants.kMinClimbPos);
	ShooterTilt shooterTiltTop = new ShooterTilt(shooter, ShooterConstants.kMaxTiltPos);
	ShooterTilt shooterTiltMid = new ShooterTilt(shooter,
			(ShooterConstants.kMaxTiltPos + ShooterConstants.kMinTiltPos) / 2.0);
	ShooterTilt shooterTiltBot = new ShooterTilt(shooter, ShooterConstants.kMinTiltPos);
	TrapperLift trapperLiftTop = new TrapperLift(trapper, TrapperConstants.kMaxLiftLen);
	TrapperLift trapperLiftMid = new TrapperLift(trapper,
			(TrapperConstants.kMinLiftLen + TrapperConstants.kMaxTiltDeg) / 2.0);
	TrapperLift trapperLiftBot = new TrapperLift(trapper, TrapperConstants.kMinLiftLen);
	TrapperTilt trapperTiltTop = new TrapperTilt(trapper, TrapperConstants.kMaxTiltDeg);
	TrapperTilt trapperTiltBot = new TrapperTilt(trapper, TrapperConstants.kMinTiltDeg);

	JustClimb climbUp = new JustClimb(climber, chassis, 0.20);
	JustClimb climbDn = new JustClimb(climber, chassis, -0.20);
	ClimberStop climbStop = new ClimberStop(climber);
	TrapperClaw trapperClawOpen = new TrapperClaw(trapper, CLAW.OPEN);
	TrapperClaw trapperClawClose = new TrapperClaw(trapper, CLAW.CLOSE);
	ShooterTiltStick shooterTiltStick = new ShooterTiltStick(shooter, m_driverController.getLeftY());

	private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");
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

		compTab.add("IntakeNote", intakeNote);
		compTab.add("ShootNote", shootNote);

		// compTab.add("Climb Up", climbUp);
		// compTab.add("Climb Dn", climbDn);
		// compTab.add("Climb Stop", climbStop);

		compTab.add("ShooterTiltTop", shooterTiltTop);
		compTab.add("ShooterTiltMid", shooterTiltMid);
		compTab.add("ShooterTiltBot", shooterTiltBot);

		shooterTab.add("ShooterTiltTop", shooterTiltTop);
		shooterTab.add("ShooterTiltMid", shooterTiltMid);
		shooterTab.add("ShooterTiltBot", shooterTiltBot);

		climberTab.add("ClimberLiftTop", climberLiftTop);
		climberTab.add("ClimberLiftMid", climberLiftMid);
		climberTab.add("ClimberLiftBot", climberLiftBot);
		
		trapperTab.add("TrapperLiftTop", trapperLiftTop);
		trapperTab.add("TrapperLiftMid", trapperLiftMid);
		trapperTab.add("TrapperLiftBot", trapperLiftBot);

		trapperTab.add("TrapperTiltTop", trapperTiltTop);
//		trapperTab.add("TrapperTiltMid", trapperTiltMid);
		trapperTab.add("TrapperTiltBot", trapperTiltBot);

		trapperTab.add("TrapperClawOpen", trapperClawOpen);
		trapperTab.add("TrapperClawClose", trapperClawClose);

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

		// shooter.setDefaultCommand(
		// new RunCommand(
		// () -> shooter.tiltTrackStick(m_operatorController.getLeftY()),
		// shooter));

		climber.setDefaultCommand(new ClimberStop(climber));
				// new RunCommand(
				// 		() -> climber.stopClimber(),
				// 		climber));
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

		// new JoystickButton(m_operatorController, Button.kX.value).debounce(1)
		// .onTrue(intakeNote);

		// new JoystickButton(m_operatorController, Button.kY.value).debounce(1)
		// .onTrue(shootNote);

		new JoystickButton(m_operatorController, Button.kRightBumper.value) // .debounce(1)
				.onTrue(shootNote);

		new JoystickButton(m_operatorController, Button.kLeftBumper.value) // .debounce(1)
				.onTrue(intakeNote);

		new JoystickButton(m_operatorController, Button.kStart.value) // .debounce(1)
				.onTrue(intakeStop);

		// new JoystickButton(m_operatorController, Button.kA.value).debounce(1)
		// .onTrue(climberLiftTop);

		// // Using Mid until Trap is working and out of way of chain
		// new JoystickButton(m_operatorController, Button.kY.value).debounce(1)
		// .onTrue(climberLiftMid);

		new JoystickButton(m_operatorController, Button.kX.value)
				.whileTrue(new RunCommand(
						() -> climber.climb(0.25),
						climber));

		new JoystickButton(m_operatorController, Button.kB.value)
				.whileTrue(new RunCommand(
						() -> climber.climb(-0.40),
						climber));
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
