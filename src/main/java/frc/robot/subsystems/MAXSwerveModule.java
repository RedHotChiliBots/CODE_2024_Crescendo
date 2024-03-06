// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.SwerveModuleConstants;

public class MAXSwerveModule {
  private final CANSparkFlex m_drivingSparkFlex;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  // --- simulation //TODO Add simulation
  // private final EncoderSim driveEncoderSim;
  // private double driveCurrentSim = 0;
  // private final EncoderSim steerEncoderSim;
  // private double steerCurrentSim = 0;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkFlex = new CANSparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkFlex.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // CAN Status frames
    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);

    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);

    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);

    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);

    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 0);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50);

    m_drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 0);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 50);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    //m_drivingEncoder = m_drivingSparkFlex.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
     m_drivingEncoder = m_drivingSparkFlex.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkFlex.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(SwerveModuleConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(SwerveModuleConstants.kTurningEncoderInverted);
    // Invert Driving and Turning motors since the MK4i flips the motor orientation
    m_turningSparkMax.setInverted(SwerveModuleConstants.kTurningMotorInverted);
    m_drivingSparkFlex.setInverted(SwerveModuleConstants.kDrivingMotorInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(SwerveModuleConstants.kDrivingP);
    m_drivingPIDController.setI(SwerveModuleConstants.kDrivingI);
    m_drivingPIDController.setD(SwerveModuleConstants.kDrivingD);
    m_drivingPIDController.setFF(SwerveModuleConstants.kDrivingFF.ks);
    m_drivingPIDController.setOutputRange(SwerveModuleConstants.kDrivingMinOutput,
        SwerveModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(SwerveModuleConstants.kTurningP);
    m_turningPIDController.setI(SwerveModuleConstants.kTurningI);
    m_turningPIDController.setD(SwerveModuleConstants.kTurningD);
    m_turningPIDController.setFF(SwerveModuleConstants.kTurningFF.ks);
    m_turningPIDController.setOutputRange(SwerveModuleConstants.kTurningMinOutput,
        SwerveModuleConstants.kTurningMaxOutput);

    m_drivingSparkFlex.setIdleMode(SwerveModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(SwerveModuleConstants.kTurningMotorIdleMode);
    m_drivingSparkFlex.setSmartCurrentLimit(SwerveModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkFlex.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);

    // --- Simulation //TODO Add Simulation
    // driveEncoderSim = new EncoderSim(driveEncoder);
    // steerEncoderSim = new EncoderSim(steerEncoder);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public double getDriveVel() {
    return m_drivingEncoder.getVelocity();
  }

  public double getDrivePosFactor() {
    return m_drivingEncoder.getPositionConversionFactor();
  }

  public double getDriveVelFactor() {
    return m_drivingEncoder.getVelocityConversionFactor();
  }

  // ----- Simulation //TODO Add Simulation

  // public void simulationUpdate(
  // double driveEncoderDist,
  // double driveEncoderRate,
  // double driveCurrent,
  // double steerEncoderDist,
  // double steerEncoderRate,
  // double steerCurrent) {
  // driveEncoderSim.setDistance(driveEncoderDist);
  // driveEncoderSim.setRate(driveEncoderRate);
  // this.driveCurrentSim = driveCurrent;
  // steerEncoderSim.setDistance(steerEncoderDist);
  // steerEncoderSim.setRate(steerEncoderRate);
  // this.steerCurrentSim = steerCurrent;
  // }

  // public double getDriveCurrentSim() {
  // return driveCurrentSim;
  // }

  // public double getSteerCurrentSim() {
  // return steerCurrentSim;
  // }
}
