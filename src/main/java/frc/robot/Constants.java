// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class CANIdConstants {
    // Drive/Turn CAN IDs
    public static final int kRearRightDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 10;
    public static final int kFrontLeftDrivingCanId = 15;
    public static final int kFrontRightDrivingCanId = 20;

    public static final int kRearRightTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 11;
    public static final int kFrontLeftTurningCanId = 16;
    public static final int kFrontRightTurningCanId = 21;

    // Climber
    public static final int kLeft1CANId = 30;
    public static final int kLeft2CANId = 31;
    public static final int kRight1CANId = 32;
    public static final int kRight2CANId = 33;

    // Feeder
    public static final int kFeederCANId = 40;

    // Intake
    public static final int kIntakeCANId = 41;

    // Shooter
    public static final int kTiltShooterCANID = 50;
    public static final int kLeftShooterCANID = 51;
    public static final int kRightShooterCANID = 52;

    // Trapper
    public static final int kLiftTrapCANId = 55;
    public static final int kTiltTrapCANId = 56;

    // CANCoders
    public static final int kTrapCANCoderID = 60;
  }

  public static final class DigitalIOConstants {
    public static final int kShooterNoteDetect = 0;
  }

  public static final class AnalogConstants {
    public static final int kShooterTiltPot = 0;
    public static final int kClimberLeftPot = 1;
    public static final int kClimberRightPot = 2;
  }

  public static final class PWMConstants {
    public static final int kLeftServoID = 0;
    public static final int kRightServoID = 1;
  }

  public static final class MotorConstants {
    public static final double kVortexFreeSpeedRpm = 6784;
    public static final double kNeoFreeSpeedRpm = 5676;
    public static final double k550FreeSpeedRpm = 11000;
  }

  public static final class ClimberConstants {
    public static final double kClimberGearRatio = 6.0;
    // public static final double kClimberRotationsPerDegree = kClimberGearRatio /
    // 360.0;
    // public static final double kClimberDegreesPerRotation = 360.0 /
    // kClimberGearRatio;

    public static final double kClimberEncoderPositionFactor = 1.0 / kClimberGearRatio;
    public static final double kClimberEncoderVelocityFactor = kClimberEncoderPositionFactor / 60.0;

    public static final boolean kLeft1Inverted = false;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double kMinOutput = 0.0;
    public static final double kMaxOutput = 0.0;

    public static final IdleMode kLeft1IdleMode = IdleMode.kBrake;
    public static final int kLeftCurrentLimit = 0;

    public static final boolean kLeft2Inverted = false;
    public static final IdleMode kLeft2IdleMode = IdleMode.kBrake;

    public static final boolean kRight1Inverted = false;
    public static final IdleMode kRight1IdleMode = IdleMode.kBrake;

    public static final boolean kRight2Inverted = false;
    public static final IdleMode kRight2IdleMode = IdleMode.kBrake;
  }

  public static final class FeederConstants {
    public static final double kFeederGearRatio = 1.0;
    // public static final double kFeederRotationsPerDegree = kFeederGearRatio /
    // 360.0;
    // public static final double kFeederDegreesPerRotation = 360.0 /
    // kFeederGearRatio;

    public static final double kFeederEncoderPositionFactor = kFeederGearRatio;
    public static final double kFeederEncoderVelocityFactor = kFeederEncoderPositionFactor / 60.0;

    public static final boolean kFeederMotorInverted = false;

    public static final double kFeederP = 0.0;
    public static final double kFeederI = 0.0;
    public static final double kFeederD = 0.0;
    public static final double kFeederFF = 0.0;
    public static final double kFeederMinOutput = 0.0;
    public static final double kFeederMaxOutput = 0.0;

    public static final IdleMode kFeederMotorIdleMode = IdleMode.kBrake;
    public static final int kFeederMotorCurrentLimit = 0;
  }

  public static final class IntakeConstants {
    public static final double kIntakeGearRatio = 1.0;
    // public static final double kIntakeRotationsPerDegree = kIntakeGearRatio /
    // 360.0;
    // public static final double kIntakeDegreesPerRotation = 360.0 /
    // kIntakeGearRatio;

    public static final double kIntakeEncoderPositionFactor = 1.0;
    public static final double kIntakeEncoderVelocityFactor = kIntakeEncoderPositionFactor / 60.0;

    public static final boolean kIntakeMotorInverted = false;

    public static final double kIntakeP = 0.0;
    public static final double kIntakeI = 0.0;
    public static final double kIntakeD = 0.0;
    public static final double kIntakeFF = 0.0;
    public static final double kIntakeMinOutput = 0.0;
    public static final double kIntakeMaxOutput = 0.0;

    public static final IdleMode kIntakeMotorIdleMode = IdleMode.kBrake;
    public static final int kIntakeMotorCurrentLimit = 0;
  }

  public static final class ShooterConstants {
    public static final double kTiltGearRatio = 25.0;
    public static final double kTiltRotationsPerDegree = kTiltGearRatio / 360.0;
    public static final double kTiltDegreesPerRotation = 360.0 / kTiltGearRatio;

    public static final double kTiltEncoderPositionFactor = 1.0 / kTiltGearRatio;
    public static final double kTiltEncoderVelocityFactor = kTiltEncoderPositionFactor / 60.0;

    public static final boolean kTiltMotorInverted = false;

    public static final double kTiltP = 0.0;
    public static final double kTiltI = 0.0;
    public static final double kTiltD = 0.0;
    public static final double kTiltFF = 0.0;
    public static final double kTiltMinOutput = 0.0;
    public static final double kTiltMaxOutput = 0.0;

    public static final IdleMode kTiltMotorIdleMode = IdleMode.kBrake;
    public static final int kTiltMotorCurrentLimit = 0;

    public static final double kLeftEncoderPositionFactor = 1.0;
    public static final double kLeftEncoderVelocityFactor = kLeftEncoderPositionFactor / 60.0;

    public static final boolean kLeftMotorInverted = false;

    public static final double kLeftP = 0.0;
    public static final double kLeftI = 0.0;
    public static final double kLeftD = 0.0;
    public static final double kLeftFF = 0.0;
    public static final double kLeftMinOutput = 0.0;
    public static final double kLeftMaxOutput = 0.0;

    public static final IdleMode kLeftMotorIdleMode = IdleMode.kBrake;
    public static final int kLeftMotorCurrentLimit = 0;

    public static final double kRightEncoderPositionFactor = 1.0;
    public static final double kRightEncoderVelocityFactor = kRightEncoderPositionFactor / 60.0;

    public static final boolean kRightMotorInverted = false;

    public static final double kRightP = 0.0;
    public static final double kRightI = 0.0;
    public static final double kRightD = 0.0;
    public static final double kRightFF = 0.0;
    public static final double kRightMinOutput = 0.0;
    public static final double kRightMaxOutput = 0.0;

    public static final IdleMode kRightMotorIdleMode = IdleMode.kBrake;
    public static final int kRightMotorCurrentLimit = 0;
  }

  public static final class TrapperConstants {
    public static final double kGripOpen = 0.0;
    public static final double kGripClose = 0.0;

    public static final double kLiftGearRatio = 5.0;
    public static final double kLiftRotationsPerDegree = kLiftGearRatio / 360.0;
    public static final double kLiftDegreesPerRotation = 360.0 / kLiftGearRatio;

    public static final double kLiftEncoderPositionFactor = 1.0 / kLiftGearRatio;
    public static final double kLiftEncoderVelocityFactor = kLiftEncoderPositionFactor / 60.0;

    public static final boolean kLiftMotorInverted = false;

    public static final double kLiftP = 0;
    public static final double kLiftI = 0;
    public static final double kLiftD = 0;
    public static final double kLiftFF = 0;
    public static final double kLiftMinOutput = 0;
    public static final double kLiftMaxOutput = 0;

    public static final IdleMode kLiftMotorIdleMode = IdleMode.kBrake;
    public static final int kLiftMotorCurrentLimit = 0;

    public static final double kTiltGearRatio = 25.0;
    public static final double kTiltRotationsPerDegree = kTiltGearRatio / 360.0;
    public static final double kTiltDegreesPerRotation = 360.0 / kTiltGearRatio;

    public static final double kTiltEncoderPositionFactor = 1.0 / kTiltGearRatio;
    public static final double kTiltEncoderVelocityFactor = kTiltEncoderPositionFactor / 60.0;

    public static final boolean kTiltMotorInverted = false;

    public static final double kTiltP = 0;
    public static final double kTiltI = 0;
    public static final double kTiltD = 0;
    public static final double kTiltFF = 0;
    public static final double kTiltMinOutput = 0;
    public static final double kTiltMaxOutput = 0;

    public static final IdleMode kTiltMotorIdleMode = IdleMode.kBrake;
    public static final int kTiltMotorCurrentLimit = 0;
  }

  public static final class ChassisConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.75);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.75);
    // Distance between front and back wheels on robot
    public static final double kWheelRadius = Math.sqrt(2.0 * Math.pow(kWheelBase, 2)) / 2.0;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kGyroReversed = false;

    // Front Left
    public static final double kFrontLeftEncoderPositionFactor = 1.0;
    public static final double kFrontLeftEncoderVelocityFactor = kFrontLeftEncoderPositionFactor / 60.0;

    public static final boolean kFrontLeftMotorInverted = false;

    public static final double kFrontLeftP = 0.0;
    public static final double kFrontLeftI = 0.0;
    public static final double kFrontLeftD = 0.0;
    public static final double kFrontLeftFF = 0.0;
    public static final double kFrontLeftMinOutput = 0.0;
    public static final double kFrontLeftMaxOutput = 0.0;

    public static final IdleMode kFrontLeftMotorIdleMode = IdleMode.kBrake;
    public static final int kFrontLeftMotorCurrentLimit = 0;

    //Front Right 
    public static final double kFrontRightEncoderPositionFactor = 1.0;
    public static final double kFrontRightEncoderVelocityFactor = kFrontLeftEncoderPositionFactor / 60.0;

    public static final boolean kFrontrightMotorInverted = false;

    public static final double kFrontRightP = 0.0;
    public static final double kFrontRightI = 0.0;
    public static final double kFrontRightD = 0.0;
    public static final double kFrontRightFF = 0.0;
    public static final double kFrontRightMinOutput = 0.0;
    public static final double kFrontRightMaxOutput = 0.0;

    public static final IdleMode kFrontRightMotorIdleMode = IdleMode.kBrake;
    public static final int kFrontRightMotorCurrentLimit = 0;

  }

  public static final class SwerveModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = false;

    public static final boolean kTurningMotorInverted = true;
    public static final boolean kDrivingMotorInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = MotorConstants.kNeoFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.feetToMeters(0.33333);// 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VisionConstants {
    public static final String kCameraName = "LimeLight";

    // Camera mounted facing forward on front perimeter and six inches off floor
    // (Note: this is the prototype robot, needs to be modified for competition
    // robot)
    public static final Transform3d kRobotToCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(30.0 / 2.0), 0.0, Units.inchesToMeters(6.0)),
        new Rotation3d(0.0, 0.0, 0.0));

    // Layout of AprilTags on the 2024 field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // Standard deviations of vision estimated poses, which affect the correction
    // rate
    // (Fake values. Experiment and determine estimation noise on actual robot)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    // Constants such as camera and target height stored. Change per robot and goal!
    public static final double kCameraHeight = Units.inchesToMeters(6.0);
    public static final double kTargetHeight = 1.32; // Tag height off floor in meters
    // Units.inchesToMeters(18.75);

    // Angle between horizontal and the camera.
    public static final double kCameraPitch = Units.degreesToRadians(0);

    // How far from the target we want to be
    public static final double kTargetDist = Units.feetToMeters(2);

    public static final double kRange2Rumble = Units.feetToMeters(20.0);

    // PID constants should be tuned per robot
    public static final double kDistP = 0.6;
    public static final double kDistI = 0.0;
    public static final double kDistD = 0.0;

    public static final double kTurnP = 0.03;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0.0;
  }
}
