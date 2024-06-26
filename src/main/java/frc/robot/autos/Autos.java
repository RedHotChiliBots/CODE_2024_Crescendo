package frc.robot.autos;

import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;

import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.AutonShootLeave;
import frc.robot.commands.AutonShootStay;
import frc.robot.commands.AutonSpeakerAmp;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Autos {
	// Define Shuffleboard tab to hold competition info
	private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

	// Define a chooser for autonomous commands
	private final SendableChooser<Command> chooser;

	// Define autonomous paths and commands
	private PathPlannerPath pathZigZag3m = null;
	private Command cmdZigZag3m = null;
	private Command cmdAutoZigZag3m = null;
	private SwerveControllerCommand swerveControllerCommand = null;
	public SwerveControllerCommand note1Command = null;
	private SwerveControllerCommand note2Command = null;
	private SwerveControllerCommand note3Command = null;

	private TrajectoryConfig config = new TrajectoryConfig(
			Constants.AutoConstants.kMaxSpeedMetersPerSecond,
			Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Constants.ChassisConstants.kDriveKinematics);

	private TrajectoryConfig configRev = new TrajectoryConfig(
			Constants.AutoConstants.kMaxSpeedMetersPerSecond,
			Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Constants.ChassisConstants.kDriveKinematics)
			.setReversed(true);

	private ProfiledPIDController thetaController = new ProfiledPIDController(
			Constants.AutoConstants.kPThetaController, 0, 0,
			Constants.AutoConstants.kThetaControllerConstraints);

	private HolonomicDriveController holonomicController = new HolonomicDriveController(
			new PIDController(Constants.AutoConstants.kPXController, 0, 0),
			new PIDController(Constants.AutoConstants.kPYController, 0, 0),
			thetaController);

	// An example trajectory to follow. All units in meters.
	public Trajectory note1Trajectory = null;
	public Trajectory note2Trajectory = null;
	public Trajectory note3Trajectory = null;

	// An example trajectory to follow. All units in meters.
	private Trajectory zigzag3Trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, new Rotation2d(0)),
			config);

	String dsEventName = DriverStation.getEventName();
	OptionalInt dsLocation = DriverStation.getLocation();
	Optional<DriverStation.Alliance> dsAlliance = DriverStation.getAlliance();
	int dsMatchNumber = DriverStation.getMatchNumber();
	double dsMatchTime = DriverStation.getMatchTime();

	public Autos(Chassis chassis, Vision vision, Intake intake, Feeder feeder, Shooter shooter) {
		System.out.println("+++++ Starting Autos Constructor +++++");

		String match = dsAlliance + " " + dsLocation + " / " + dsEventName + " " + dsMatchNumber;

		final GenericEntry sbMatch = compTab.addPersistent("Match Info", "")
				.withWidget("Text View").withPosition(0, 1).withSize(2, 1).getEntry();

		sbMatch.getString(match);

		// Game Manual page 45
		// Note 1
		note1Trajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				chassis.getPose(), // start from current pose
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(new Translation2d(2.9, 1.0)),
				// End 3 meters straight ahead of where we started, facing forward
				new Pose2d(2.9, 1.45, new Rotation2d(Math.toRadians(90.0))),
				configRev);

		note2Trajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				chassis.getPose(), // start from current pose
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(new Translation2d(2.9, 2.0)),
				// End 3 meters straight ahead of where we started, facing forward
				new Pose2d(2.9, 2.9, new Rotation2d(Math.toRadians(90.0))),
				configRev);

		note3Trajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				chassis.getPose(), // start from current pose
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(new Translation2d(2.9, 3.0)),
				// End 3 meters straight ahead of where we started, facing forward
				new Pose2d(2.9, 4.35, new Rotation2d(Math.toRadians(90.0))),
				configRev);

		note1Command = new SwerveControllerCommand(
				note1Trajectory,
				chassis::getPose,
				ChassisConstants.kDriveKinematics,
				holonomicController,
				chassis::setModuleStates,
				chassis);

		note2Command = new SwerveControllerCommand(
				note1Trajectory,
				chassis::getPose,
				ChassisConstants.kDriveKinematics,
				holonomicController,
				chassis::setModuleStates,
				chassis);

		note3Command = new SwerveControllerCommand(
				note1Trajectory,
				chassis::getPose,
				ChassisConstants.kDriveKinematics,
				holonomicController,
				chassis::setModuleStates,
				chassis);

		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		swerveControllerCommand = new SwerveControllerCommand(
				zigzag3Trajectory,
				chassis::getPose,
				ChassisConstants.kDriveKinematics,
				holonomicController,
				chassis::setModuleStates,
				chassis);

		// Configure the AutoBuilder fpr Swerve
		AutoBuilder.configureHolonomic(
				chassis::getPose, // Robot pose supplier
				chassis::resetPose, // Method to reset odometry (will be called if your auto has a
				// starting pose)
				chassis::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				chassis::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
				// ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
						// in your Constants class
						new PIDConstants(SwerveModuleConstants.kDrivingP,
								0.0, 0.0), // Translation PID constants
						new PIDConstants(SwerveModuleConstants.kDrivingP,
								0.0, 0.0), // Rotation PID constants
						ChassisConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
						ChassisConstants.kWheelRadius, // Drive base radius in meters. Distance
						// from robot center to furthest module.
						new ReplanningConfig() // Default path replanning config. See the API
														// for the options here
				),
				chassis::getFlipPath,
				chassis // Reference to this subsystem to set requirements
		);

		// ********************************************
		// Register Named Commands
		// Note: Must be done before defining Auto commands
		// NamedCommands.registerCommand("ZigZag3m", cmdZigZag3m);

		String temp = AutoBuilder.isConfigured() ? "IS" : "IS NOT";
		System.out.println("AutoBuilder " + temp + " configured");
		temp = AutoBuilder.isPathfindingConfigured() ? "IS" : "IS NOT";
		System.out.println("AutoBuilder Pathfinding " + temp + " configured");

		// ********************************************
		// Generate Paths and Path commands
		// pathZigZag3m = PathPlannerPath.fromPathFile("ZigZag3m");
		// cmdZigZag3m = AutoBuilder.followPathWithEvents(pathZigZag3m);

		// ********************************************
		// Generate Auto commands
		// Note: Named commands used in Auto command must be defined
		// before defining the Auto command
		AutonShootLeave autoShootLeave = new AutonShootLeave(chassis, intake, feeder, shooter);
		AutonShootStay autoShootStay = new AutonShootStay(chassis, intake, feeder, shooter);
		AutonSpeakerAmp autoSpeakerAmp = new AutonSpeakerAmp(chassis, this, vision, intake, feeder, shooter);

		// ********************************************
		// Initialize auto command chooser with auton commands
		chooser = AutoBuilder.buildAutoChooser();

		chooser.setDefaultOption("Shoot N Leave", autoShootLeave);
		chooser.addOption("Shoot N Stay", autoShootStay);
		chooser.addOption("Speaker Amp", autoSpeakerAmp);
		// chooser.addOption("Auto ZigZag3Cmd", cmdAutoZigZag3m);

		// ********************************************
		// Add Auton Command chooser to Shuffleboard
		compTab.add("Auton Command", chooser)
				.withWidget("ComboBox Chooser")
				.withPosition(0, 10)
				.withSize(4, 1);

		System.out.println("----- Ending Autos Constructor -----");
	}

	public SendableChooser<Command> getChooser() {
		return chooser;
	}
}