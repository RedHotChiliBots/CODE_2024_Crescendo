
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.hal.SimDevice;  //TODO Add Simulation
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;  //TODO Remove Odometry
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.system.plant.DCMotor; //TODO Add Simulation

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.WPIUtilJNI;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro; //TODO Add Simulation
//import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim; //TODO Add Simulation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.utils.SwerveUtils;

public class Chassis extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      CANIdConstants.kFrontLeftDrivingCanId,
      CANIdConstants.kFrontLeftTurningCanId,
      ChassisConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      CANIdConstants.kFrontRightDrivingCanId,
      CANIdConstants.kFrontRightTurningCanId,
      ChassisConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      CANIdConstants.kRearLeftDrivingCanId,
      CANIdConstants.kRearLeftTurningCanId,
      ChassisConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      CANIdConstants.kRearRightDrivingCanId,
      CANIdConstants.kRearRightTurningCanId,
      ChassisConstants.kBackRightChassisAngularOffset);

  // ==============================================================
  // Initialize NavX AHRS board
  // Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
  private AHRS ahrs = null;

  private double pitchOffset = 0.0;
  private double rollOffset = 0.0;

  private PowerDistribution pdh = new PowerDistribution(CANIdConstants.kPDHCanID, ModuleType.kRev);

  // The gyro sensor
  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(ChassisConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(ChassisConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // The robot pose estimator for tracking swerve odometry and applying vision
  // corrections.
  private final SwerveDrivePoseEstimator poseEstimator;

  // private final SwerveDriveSim swerveDriveSim; //TODO Add Simulation

  // private final ADXRS450_GyroSim gyroSim;
  // private final SimDevice gyroSim;

  // Odometry class for tracking robot pose
  // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry( //TODO Remove
  // Odometry
  // ChassisConstants.kDriveKinematics,
  // Rotation2d.fromDegrees(-ahrs.getAngle()),
  // new SwerveModulePosition[] {
  // m_frontLeft.getPosition(),
  // m_frontRight.getPosition(),
  // m_rearLeft.getPosition(),
  // m_rearRight.getPosition()
  // });

  // ==============================================================
  // Define Shuffleboard data - Chassis Tab
  private final ShuffleboardTab chassisTab = Shuffleboard.getTab("Chassis");
  private final GenericEntry sbAngle = chassisTab.addPersistent("Angle", 0)
      .withWidget("Text View").withPosition(3, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbFusedHeading = chassisTab.addPersistent("FusedHeading", 0)
      .withWidget("Text View").withPosition(3, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbCompassHeading = chassisTab.addPersistent("CompassHeading", 0)
      .withWidget("Text View").withPosition(3, 2).withSize(2, 1).getEntry();
  private final GenericEntry sbRotDegree = chassisTab.addPersistent("Rotation2d", 0)
      .withWidget("Text View").withPosition(3, 3).withSize(2, 1).getEntry();
  private final GenericEntry sbPitch = chassisTab.addPersistent("Pitch", 0)
      .withWidget("Text View").withPosition(5, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbRoll = chassisTab.addPersistent("Roll", 0)
      .withWidget("Text View").withPosition(5, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbYaw = chassisTab.addPersistent("Yaw", 0)
      .withWidget("Text View").withPosition(5, 2).withSize(2, 1).getEntry();

  /** Creates a new DriveSubsystem. */
  public Chassis() {
    System.out.println("+++++ Starting Chassis Constructor +++++");

    // ==============================================================
    // Initialize Rev PDH
    pdh.clearStickyFaults();

    // ==============================================================
    // Initialize NavX AHRS board
    // Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
    try {
      ahrs = new AHRS(SPI.Port.kMXP, (byte) 100); // 100 Hz
      ahrs.enableBoardlevelYawReset(true);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    // Define the standard deviations for the pose estimator, which determine how
    // fast the pose estimate converges to the vision measurement. This should
    // depend on the vision measurement noise and how many or how frequently
    // vision measurements are applied to the pose estimator.
    Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    Vector<N3> visionStdDevs = VecBuilder.fill(1, 1, 1);

    poseEstimator = new SwerveDrivePoseEstimator(
        ChassisConstants.kDriveKinematics,
        getRotation2d(),
        getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionStdDevs);

    // ----- Simulation
    // ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    // gyroSim = new ADXRS450_GyroSim(gyro);
    // // gyroSim = new SimDevice();
    // swerveDriveSim = new SwerveDriveSim(
    // SwerveModuleConstants.kDrivingFF,
    // DCMotor.getFalcon500(1),
    // ChassisConstants.kDriveGearRatio,
    // SwerveModuleConstants.kWheelDiameterMeters / 2.0,
    // SwerveModuleConstants.kTurningFF,
    // DCMotor.getFalcon500(1),
    // ChassisConstants.kSteerGearRatio,
    // ChassisConstants.kDriveKinematics);

    ahrs.reset();
    zeroYaw();
    resetPose(getPose());

    setChannelOff();

    pitchOffset = Math.toRadians(-getPitch());
    rollOffset = Math.toRadians(-getRoll());

    System.out.println("----- Ending Chassis Constructor -----");
  }

  @Override
  public void periodic() {
    sbAngle.setDouble(getAngle());
    sbYaw.setDouble(getYaw());
    sbPitch.setDouble(getPitch());
    sbRoll.setDouble(getRoll());
    sbFusedHeading.setDouble(getFusedHeading());
    sbCompassHeading.setDouble(getCompassHeading());
    sbRotDegree.setDouble(getRotation2d().getDegrees());

    // Update the odometry of the swerve drive using the wheel encoders and gyro.
    poseEstimator.update(getRotation2d(), getModulePositions());

    // Update the odometry in the periodic block
    // m_odometry.update( //TODO Remove Odometry
    // Rotation2d.fromDegrees(-ahrs.getAngle()),
    // new SwerveModulePosition[] {
    // m_frontLeft.getPosition(),
    // m_frontRight.getPosition(),
    // m_rearLeft.getPosition(),
    // m_rearRight.getPosition()
    // });

    // Add Swerve Drive to SmartDashboard
    SwerveModuleState[] currentStates = new SwerveModuleState[4];

    currentStates[0] = m_frontLeft.getState();
    currentStates[1] = m_frontRight.getState();
    currentStates[2] = m_rearLeft.getState();
    currentStates[3] = m_rearRight.getState();

    double[] stateAdv = new double[8];

    for (int i = 0; i < 4; i++) {
      stateAdv[2 * i] = currentStates[i].angle.getDegrees();
      stateAdv[2 * i + 1] = currentStates[i].speedMetersPerSecond;
    }

    SmartDashboard.putNumberArray("swerve/status", stateAdv);
    SmartDashboard.putNumber("swerve/velocity-rpm", m_frontLeft.getDriveVel());
    SmartDashboard.putNumber("swerve/posfactorC", SwerveModuleConstants.kDrivingEncoderPositionFactor);
    SmartDashboard.putNumber("swerve/velfactorC", SwerveModuleConstants.kDrivingEncoderVelocityFactor);
    SmartDashboard.putNumber("swerve/posfactorE", m_frontLeft.getDrivePosFactor());
    SmartDashboard.putNumber("swerve/velfactorE", m_frontLeft.getDriveVelFactor());
//    SmartDashboard.putBoolean("pdh/channel", pdh.getSwitchableChannel());
  }

  public void setChannelOn() {
    pdh.setSwitchableChannel(true);
  }

  public void setChannelOff() {
    pdh.setSwitchableChannel(false);
  }

  /**
   * Get the SwerveModulePosition of each swerve module (position, angle). The
   * returned array order
   * matches the kinematics module order.
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  /**
   * See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
   */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  /**
   * See
   * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
   */
  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  /**
   * Reset the estimated pose of the swerve drive on the field.
   *
   * @param pose         New robot pose.
   * @param resetSimPose If the simulated robot pose should also be reset. This
   *                     effectively
   *                     teleports the robot and should only be used during the
   *                     setup of the simulation world.
   */
  public void resetPose(Pose2d pose) {
    resetPose(pose, false);
  }

  public void resetPose(Pose2d pose, boolean resetSimPose) { // TODO Add Simulation
    // if (resetSimPose) {
    // swerveDriveSim.reset(pose, false);
    // // we shouldnt realistically be resetting pose after startup, but we will
    // handle
    // // it anyway for testing
    // // for (int i = 0; i < swerveMods.length; i++) {
    // // swerveMods[i].simulationUpdate(0, 0, 0, 0, 0, 0);
    // // }
    // m_frontLeft.simulationUpdate(0, 0, 0, 0, 0, 0);
    // m_frontRight.simulationUpdate(0, 0, 0, 0, 0, 0);
    // m_rearLeft.simulationUpdate(0, 0, 0, 0, 0, 0);
    // m_rearRight.simulationUpdate(0, 0, 0, 0, 0, 0);

    // gyroSim.setAngle(-pose.getRotation().getDegrees());
    // gyroSim.setRate(0);
    // }

    poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /** Get the estimated pose of the swerve drive on the field. */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Raw gyro yaw (this may not match the field heading!). */
  public double getYaw() {
    return ahrs.getYaw();
  }

  public double getPitch() {
    return Math.toDegrees(SwerveUtils.WrapAngle(Math.toRadians(ahrs.getPitch()) + pitchOffset));
  }

  public double getRoll() {
    return Math.toDegrees(SwerveUtils.WrapAngle(Math.toRadians(-ahrs.getRoll()) + rollOffset));
  }

  public Rotation2d getRotation2d() {
    return ahrs.getRotation2d();
  }

  public double getCompassHeading() {
    return ahrs.getCompassHeading();
  }

  public double getFusedHeading() {
    return ahrs.getFusedHeading();
  }

  public double getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return ahrs.getAngle();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  // public Pose2d getPose() { //TODO Remove Odometry
  // return m_odometry.getPoseMeters();
  // }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  // public void resetPose(Pose2d pose) { //TODO Remove Odometry
  // m_odometry.resetPosition(
  // Rotation2d.fromDegrees(-ahrs.getAngle()),
  // new SwerveModulePosition[] {
  // m_frontLeft.getPosition(),
  // m_frontRight.getPosition(),
  // m_rearLeft.getPosition(),
  // m_rearRight.getPosition()
  // },
  // pose);
  // }

  /**
   * 
   * @param xSpeed
   * @param ySpeed
   * @param rot
   * @param fieldRelative
   * @param rateLimit
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    // Convert module states to chassis speeds
    ChassisSpeeds chassisSpeeds = ChassisConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState());

    return chassisSpeeds;
  }

  /**
   * 
   * @param xSpeed
   * @param ySpeed
   * @param rot
   * @param fieldRelative
   * @param rateLimit
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates = ChassisConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, ChassisConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  private boolean flipPath = false;

  public void setFlipPath(boolean fp) {
    flipPath = fp;
  }

  public boolean getFlipPath() {
    return flipPath;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(ChassisConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * ChassisConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * ChassisConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * ChassisConstants.kMaxAngularSpeed;

    var swerveModuleStates = ChassisConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-ahrs.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, ChassisConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Stops wheels.
   */
  public void stopChassis() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, m_frontLeft.getState().angle));
    m_frontRight.setDesiredState(new SwerveModuleState(0, m_frontRight.getState().angle));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, m_rearLeft.getState().angle));
    m_rearRight.setDesiredState(new SwerveModuleState(0, m_rearRight.getState().angle));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, ChassisConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroYaw() {
    ahrs.zeroYaw();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -ahrs.getRate() * (ChassisConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
