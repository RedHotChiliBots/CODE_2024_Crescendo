// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import com.revrobotics.CANSparkBase.IdleMode;

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

  public static final class ANSIConstants {
    public static final String ANSI_RESET = "\u001B[0m";
    public static final String ANSI_BLACK = "\u001B[30m";
    public static final String ANSI_RED = "\u001B[31m";
    public static final String ANSI_GREEN = "\u001B[32m";
    public static final String ANSI_YELLOW = "\u001B[33m";
    public static final String ANSI_BLUE = "\u001B[34m";
    public static final String ANSI_PURPLE = "\u001B[35m";
    public static final String ANSI_CYAN = "\u001B[36m";
    public static final String ANSI_WHITE = "\u001B[37m";

    public static final String ANSI_BLACK_BACKGROUND = "\u001B[40m";
    public static final String ANSI_RED_BACKGROUND = "\u001B[41m";
    public static final String ANSI_GREEN_BACKGROUND = "\u001B[42m";
    public static final String ANSI_YELLOW_BACKGROUND = "\u001B[43m";
    public static final String ANSI_BLUE_BACKGROUND = "\u001B[44m";
    public static final String ANSI_PURPLE_BACKGROUND = "\u001B[45m";
    public static final String ANSI_CYAN_BACKGROUND = "\u001B[46m";
    public static final String ANSI_WHITE_BACKGROUND = "\u001B[47m";
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class CANIdConstants {
    public static final int kPDHCanID = 1;

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
    public static final int kLeft1CANId = 31;
    public static final int kLeft2CANId = 30;
    public static final int kRight1CANId = 33;
    public static final int kRight2CANId = 32;

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
    public static final int kIntakeNoteDetect = 0;
    public static final int kIntakeNoteCapture = 1;
  }

  public static final class AnalogConstants {
    public static final int kShooterTiltPot = 0;
    public static final int kClimberLeftPot = 1;
    public static final int kClimberRightPot = 2;
    public static final int kTrapperLiftPot = 3;
  }

  public static final class PWMConstants {
    public static final int kTopServoID = 0;
    public static final int kBotServoID = 1;
  }

  public static final class MotorConstants {
    public static final double kVortexFreeSpeedRpm = 6784;
    public static final double kNeoFreeSpeedRpm = 5676;
    public static final double k550FreeSpeedRpm = 11000;
  }

  public static final class ClimberConstants {

    public static final double kGearRatio = 6.0;
    public static final double kMaxShaftRevs = 10.0;
    public static final double kMaxMotorRevs = kMaxShaftRevs * kGearRatio;
    public static final double kMaxPotVolt = 3.3;
    public static final double kMinPotVolt = 0.0;

    public static final double kRightPotMin = 1.0;
    public static final double kLeftPotMin = 1.0;
    public static final double kMaxClimbPos = 18.0;
    public static final double kMidClimbPos = (ClimberConstants.kMaxClimbPos + ClimberConstants.kMinClimbPos) / 2.0;
    public static final double kMinClimbPos = 0.0;

    // public static final double kBarrelDia = 0.75;
    // public static final double kDistPerShaftRev = kBarrelDia * Math.PI;
    
    public static final double kClimbBarrelDia = 0.75;
    public static final double kInchPerRev = (Math.PI * kClimbBarrelDia) / kGearRatio;

    // public static final double kDistPerMotorRev = kDistPerShaftRev / kGearRatio;
    // public static final double kShaftRevsPerVolt = kMaxShaftRevs / kMaxPotVolt;
    // public static final double kMotorRevsPerVolt = kMaxMotorRevs / kMaxPotVolt;
    // public static final double kDistPerVolt = kDistPerMotorRev * kMotorRevsPerVolt;
    // public static final double kMaxShaftRot = kMaxClimbPos / kDistPerShaftRev;
    // public static final double kMaxMotorRot = kMaxClimbPos / kDistPerMotorRev;

    public static final double kClimberEncoderPositionFactor = kInchPerRev;
//    public static final double kClimberEncoderPositionFactor = kDistPerVolt;
    public static final double kClimberTollerance = 1.0;

    public static final int kCurrentLimit = 40;

    public static final double kP = 0.0275;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;

    public static final boolean kLeft1Inverted = true;
    public static final IdleMode kLeft1IdleMode = IdleMode.kBrake;
    public static final boolean kLeft2Inverted = true;
    public static final IdleMode kLeft2IdleMode = IdleMode.kBrake;

    public static final boolean kRight1Inverted = true;
    public static final IdleMode kRight1IdleMode = IdleMode.kBrake;
    public static final boolean kRight2Inverted = true;
    public static final IdleMode kRight2IdleMode = IdleMode.kBrake;
  }

  public static final class FeederConstants {
    public static final double kMaxFeederVel = MotorConstants.kNeoFreeSpeedRpm;
    public static final double kMinFeederVel = -MotorConstants.kNeoFreeSpeedRpm;

    public static final double kFeederVelocity = MotorConstants.kNeoFreeSpeedRpm * 0.8;

    public static final double kFeederGearRatio = 1.0;
    // public static final double kFeederRotationsPerDegree = kFeederGearRatio /
    // 360.0;
    // public static final double kFeederDegreesPerRotation = 360.0 /
    // kFeederGearRatio;

    public static final double kFeederEncoderPositionFactor = kFeederGearRatio;
    public static final double kFeederEncoderVelocityFactor = kFeederEncoderPositionFactor / 60.0;

    public static final boolean kFeederMotorInverted = true;

    public static final double kFeederP = 0.1;
    public static final double kFeederI = 0.0;
    public static final double kFeederD = 0.0;
    public static final double kFeederIz = 0.0;
    public static final double kFeederFF = 0.0;
    public static final double kFeederMinOutput = -1.0;
    public static final double kFeederMaxOutput = 1.0;

    public static final IdleMode kFeederMotorIdleMode = IdleMode.kBrake;
    public static final int kFeederMotorCurrentLimit = 40;
  }

  public static final class IntakeConstants {
    public static final double kMaxIntakeVel = MotorConstants.kNeoFreeSpeedRpm;
    public static final double kMinIntakeVel = -MotorConstants.kNeoFreeSpeedRpm;
    public static final double kIntakeVelocity = MotorConstants.kNeoFreeSpeedRpm * 0.8;

    public static final double kIntakeGearRatio = 1.0;
    // public static final double kIntakeRotationsPerDegree = kIntakeGearRatio /
    // 360.0;
    // public static final double kIntakeDegreesPerRotation = 360.0 /
    // kIntakeGearRatio;

    public static final double kIntakeEncoderPositionFactor = 1.0;
    public static final double kIntakeEncoderVelocityFactor = kIntakeEncoderPositionFactor / 60.0;

    public static final boolean kIntakeMotorInverted = true;

    public static final double kIntakeP = 1.0; // 0.00008;
    public static final double kIntakeI = 0.0;
    public static final double kIntakeD = 0.0;
    public static final double kIntakeFF = 0.0;
    public static final double kIntakeMinOutput = -1.0;
    public static final double kIntakeMaxOutput = 1.0;

    public static final IdleMode kIntakeMotorIdleMode = IdleMode.kBrake;
    public static final int kIntakeMotorCurrentLimit = 40;
  }

  public static final class ShooterConstants {
    public static final double kMaxShootRPM = MotorConstants.kVortexFreeSpeedRpm;
    public static final double kMinShootRPM = -MotorConstants.kVortexFreeSpeedRpm;
    public static final double kShootVelocity = MotorConstants.kVortexFreeSpeedRpm * 0.9;
    public static final double kMoveVelocity = MotorConstants.kVortexFreeSpeedRpm * 0.5;

    public static final double kMaxTiltDeg = 65.0;
    public static final double kMidTiltDeg = (ShooterConstants.kMaxTiltPos + ShooterConstants.kMinTiltPos) / 2.0;
    public static final double kMinTiltDeg = 30.0;
    public static final double kTiltTollerance = 0.5;
    public static final double kShootTollerance = 50.0;

    public static final double kGearRatio = 25.0;
    public static final double kMaxShaftRevs = 9.0;
    public static final double kMaxMotorRevs = kMaxShaftRevs * kGearRatio;
    public static final double kMaxPotVolt = 2.86;
    public static final double kMinPotVolt = 0.0390625;

    public static final double kTiltPotAdj = 2.973;
    public static final double kMaxTiltPos = 4.875;
    public static final double kMinTiltPos = 0.0;
    public static final double kMidTiltPos = (ShooterConstants.kMaxTiltPos + ShooterConstants.kMinTiltPos) / 2.0;

    public static final double kSprocketDia = 1.44;
    public static final double kDistPerShaftRev = kSprocketDia * Math.PI;
    public static final double kDistPerMotorRev = kDistPerShaftRev / kGearRatio;
    public static final double kShaftRevsPerVolt = kMaxShaftRevs / kMaxPotVolt;
    public static final double kMotorRevsPerVolt = kMaxMotorRevs / kMaxPotVolt;
    public static final double kDistPerVolt = 13.58;  //kDistPerMotorRev * kMotorRevsPerVolt;
    public static final double kVoltPerDist = 1.0 / kDistPerVolt;
    public static final double kMaxShaftRot = kMaxTiltPos / kDistPerShaftRev;
    public static final double kMaxMotorRot = kMaxTiltPos / kDistPerMotorRev;

    public static final double kTiltEncoderPositionFactor = kDistPerVolt;

    // public static final double kTiltVoltsPerDeg =
    // Math.toDegrees(Math.asin(RobotContainer.shooter.getTiltVoltage() / 12.0));

    public static final double kTiltEncoderVelocityFactor = kTiltEncoderPositionFactor / 60.0;

    public static final boolean kTiltMotorInverted = true;

    public static final double kTiltP = 0.5;
    public static final double kTiltI = 0.0;
    public static final double kTiltD = 0.0;
    public static final double kTiltFF = 0.0;
    public static final double kTiltMinOutput = -1.0;
    public static final double kTiltMaxOutput = 1.0;

    public static final IdleMode kTiltMotorIdleMode = IdleMode.kBrake;
    public static final int kTiltMotorCurrentLimit = 20;

    public static final double kLeftEncoderPositionFactor = 1.0;
    public static final double kLeftEncoderVelocityFactor = kLeftEncoderPositionFactor / 60.0;

    public static final boolean kLeftMotorInverted = true;

    public static final double kLeftP = 1.0;
    public static final double kLeftI = 0.0;
    public static final double kLeftD = 0.0;
    public static final double kLeftIz = 0.0;
    public static final double kLeftFF = 0.0;
    public static final double kLeftMinOutput = -1.0;
    public static final double kLeftMaxOutput = 1.0;

    public static final IdleMode kLeftMotorIdleMode = IdleMode.kBrake;
    public static final int kLeftMotorCurrentLimit = 40;

    public static final double kRightEncoderPositionFactor = 1.0;
    public static final double kRightEncoderVelocityFactor = kRightEncoderPositionFactor / 60.0;

    public static final boolean kRightMotorInverted = true;

    public static final IdleMode kRightMotorIdleMode = IdleMode.kBrake;
    public static final int kRightMotorCurrentLimit = 40;
  }

  public static final class TrapperConstants {
    public static final double kRangePos = 20.0; // inches
    public static final double kOffsetPos = -1.0; // inches

    public static final double kMaxTiltDeg = 180.0;
    public static final double kMidTiltDeg = (TrapperConstants.kMinTiltDeg + TrapperConstants.kMaxTiltDeg) / 2.0;
    public static final double kMinTiltDeg = 90.0;
    public static final double kTiltTollerance = 0.5;

    public static final double kLiftPotAdj = 4.5;
    public static final double kMaxLiftLen = 19.5;
    public static final double kMidLiftLen = (TrapperConstants.kMinLiftLen + TrapperConstants.kMaxLiftLen) / 2.0;
    public static final double kMinLiftLen = 0.0;
    public static final double kLiftTollerance = 0.25;

    // public static final double kMaxClawDeg = 90.0;
    // public static final double kMinClawDeg = 0.0;

    public static final double kTopClawOpen = 0.0;
    public static final double kTopClawClose = 185.0;
    public static final double kBotClawOpen = 185.0;
    public static final double kBotClawClose = 0.0;

    public static final double kMaxShaftRevs = 10.0;
    public static final double kMaxPotVolt = 3.3;
    public static final double kMinPotVolt = 0.0;

    public static final double kBarrelDia = 1.0;
    public static final double kLiftGearRatio = 5.0;
    public static final double kMaxMotorRevs = kMaxShaftRevs * kLiftGearRatio;

    public static final double kDistPerShaftRev = Math.PI * kBarrelDia;
    public static final double kDistPerMotorRev = kDistPerShaftRev / kLiftGearRatio;
    public static final double kShaftRevsPerVolt = kMaxShaftRevs / kMaxPotVolt;
    public static final double kMotorRevsPerVolt = kMaxMotorRevs / kMaxPotVolt;
    public static final double kDistPerVolt = kDistPerMotorRev * kMotorRevsPerVolt;
    public static final double kMaxShaftRot = kMaxLiftLen / kDistPerShaftRev;
    public static final double kMaxMotorRot = kMaxLiftLen / kDistPerMotorRev;

    public static final double kLiftEncoderPositionFactor = kDistPerVolt;

    public static final boolean kLiftMotorInverted = true;

    public static final double kLiftP = 0.079;
    public static final double kLiftI = 0.000001;
    public static final double kLiftD = 0;
    public static final double kLiftFF = 0;
    public static final double kLiftMinOutput = -1.0;
    public static final double kLiftMaxOutput = 1.0;

    public static final IdleMode kLiftMotorIdleMode = IdleMode.kBrake;
    public static final int kLiftMotorCurrentLimit = 40;

    public static final double kTiltGearRatio = 100.0;
    public static final double kTiltRotationsPerDegree = kTiltGearRatio / 360.0;
    public static final double kTiltDegreesPerRotation = 360.0 / kTiltGearRatio;

    public static final double kTiltEncoderPositionFactor = 2.0 * Math.PI;
    public static final double kTiltEncoderVelocityFactor = kTiltEncoderPositionFactor / 60.0;

    public static final boolean kTiltEncoderInverted = true;
    public static final double kTiltZeroOffset = Math.toRadians(247.0);

    public static final boolean kTiltMotorInverted = false;

    public static final double kTiltP = 0.115;
    public static final double kTiltI = 0.00005;
    public static final double kTiltD = 0;
    public static final double kTiltFF = 0;
    public static final double kTiltMinOutput = -1.0;
    public static final double kTiltMaxOutput = 1.0;

    public static final IdleMode kTiltMotorIdleMode = IdleMode.kBrake;
    public static final int kTiltMotorCurrentLimit = 40;
  }

  public static final class ChassisConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(19.3 * 3.0);
    public static final double kMaxAngularSpeed = 12 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    public static final double kDriveGearRatio = 1.0;
    public static final double kSteerGearRatio = 1.0;

    public static final double kTrackWidth = Units.inchesToMeters(25.75);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.00);
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

    // Front Right
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

    public static final double kWheelCirc = (Math.PI * SwerveModuleConstants.kWheelDiameterMeters); // meters
    public static final int kEncoderResolution = 1; // not used, NEO's native units are rotations
    public static final double kGearBoxRatio = 6.12;
    public static final double kPosFactor = kWheelCirc / kGearBoxRatio; // Meters per Revolution
    public static final double kVelFactor = kWheelCirc / kGearBoxRatio / 60.0; // Meters per Second

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
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    // public static final double kDrivingMotorReduction = (45.0 * 22) /
    // (kDrivingMotorPinionTeeth * 15);
    // public static final double kDrivingMotorReduction = 6.75; // L2
    public static final double kDrivingMotorReduction = 6.12; // L3

    public static final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters / kDrivingMotorReduction; // meters
                                                                                                                   // /
                                                                                                                   // rev
    public static final double kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0; // meters per
                                                                                                     // second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final SimpleMotorFeedforward kDrivingFF = new SimpleMotorFeedforward(0, 0, 0); // 1 /
                                                                                                 // kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final SimpleMotorFeedforward kTurningFF = new SimpleMotorFeedforward(0, 0, 0);
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

    // Camera mounted facing forward on front perimeter
    // The following number are derived from the CAD model
    // 21.5" off floor, 8.96" left of center, 13.67 forward of center
    // Limelight 2 FOV 59.6 x 49.7 degrees
    public static final Transform3d kRobotToCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(8.96),
            Units.inchesToMeters(13.67),
            Units.inchesToMeters(21.5)),
        new Rotation3d(0.0, 49.7 / 2.0, 0.0));

    // Layout of AprilTags on the 2024 field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // Standard deviations of vision estimated poses, which affect the correction rate
    // (Fake values. Experiment and determine estimation noise on actual robot)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    // Constants such as camera and target height stored. Change per robot and goal!
    public static final double kCameraHeight = Units.inchesToMeters(21.5);
    public static final double kTargetHeight = 1.32; // Tag height off floor in meters
    // Units.inchesToMeters(18.75);

    // Angle between horizontal and the camera.
    public static final double kCameraPitch = Units.degreesToRadians(49.7 / 2.0);

    // How far from the target we want to be
    public static final double kTargetDist = Units.feetToMeters(2.0);

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
