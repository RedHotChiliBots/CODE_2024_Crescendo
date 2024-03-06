// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.DigitalIOConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  private final GenericEntry sbVel = shooterTab.addPersistent("Velocity", 0)
      .withWidget("Text View").withPosition(2, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbVelSP = shooterTab.addPersistent("Velocity SP", 0)
      .withWidget("Text View").withPosition(4, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbPos = shooterTab.addPersistent("Position", 0)
      .withWidget("Text View").withPosition(2, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbPosSP = shooterTab.addPersistent("Position SP", 0)
      .withWidget("Text View").withPosition(4, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbTiltPos = shooterTab.addPersistent("Tilt Pos", 0)
      .withWidget("Text View").withPosition(2, 2).withSize(2, 1).getEntry();
  private final GenericEntry sbTiltVolt = shooterTab.addPersistent("Tilt Volt", 0)
      .withWidget("Text View").withPosition(2, 3).withSize(2, 1).getEntry();
  private final GenericEntry sbTiltSP = shooterTab.addPersistent("Tilt SP", 0)
      .withWidget("Text View").withPosition(4, 2).withSize(2, 1).getEntry();
  private final GenericEntry sbNote = shooterTab.addPersistent("Note Detect", false)
      .withWidget("Boolean Box").withPosition(0, 0).withSize(1, 1).getEntry();

  private final CANSparkMax tilt = new CANSparkMax(CANIdConstants.kTiltShooterCANID, MotorType.kBrushless);
  private final CANSparkFlex leader = new CANSparkFlex(CANIdConstants.kLeftShooterCANID, MotorType.kBrushless);
  private final CANSparkFlex follower = new CANSparkFlex(CANIdConstants.kRightShooterCANID, MotorType.kBrushless);

  private final RelativeEncoder leaderEncoder = leader.getEncoder();
  private final SparkPIDController leaderPIDController = leader.getPIDController();

  private final SparkAnalogSensor tiltEncoder = tilt.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
  private final SparkPIDController tiltPIDController = tilt.getPIDController();

  private final DigitalInput noteDetect = new DigitalInput(DigitalIOConstants.kShooterNoteDetect);

  private double tiltSetPoint = 0.0;
  private double velSetPoint = 0.0;
  private double posSetPoint = 0.0;

  public Shooter() {
    System.out.println("+++++ Starting Shooter Constructor +++++");

    tilt.restoreFactoryDefaults();
    tilt.clearFaults();
    leader.restoreFactoryDefaults();
    leader.clearFaults();
    follower.restoreFactoryDefaults();
    follower.clearFaults();

    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);
    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 0);
    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 0);

    leader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    leader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    leader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    leader.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);
    leader.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    leader.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 0);
    leader.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 0);

    follower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    follower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    follower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
    follower.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);
    follower.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    follower.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 0);
    follower.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 0);

    tiltEncoder.setPositionConversionFactor(ShooterConstants.kTiltEncoderPositionFactor);
    // tiltEncoder.setVelocityConversionFactor(ShooterConstants.kTiltEncoderVelocityFactor);

    tiltPIDController.setFeedbackDevice(tiltEncoder);
    tiltPIDController.setP(ShooterConstants.kTiltP);
    tiltPIDController.setI(ShooterConstants.kTiltI);
    tiltPIDController.setD(ShooterConstants.kTiltD);
    tiltPIDController.setFF(ShooterConstants.kTiltFF);
    tiltPIDController.setOutputRange(ShooterConstants.kTiltMinOutput,
        ShooterConstants.kTiltMaxOutput);

    tilt.setInverted(ShooterConstants.kTiltMotorInverted);
    tilt.setIdleMode(ShooterConstants.kTiltMotorIdleMode);
    tilt.setSmartCurrentLimit(ShooterConstants.kTiltMotorCurrentLimit);

    tilt.burnFlash();

    leaderEncoder.setPositionConversionFactor(ShooterConstants.kLeftEncoderPositionFactor);
    leaderEncoder.setVelocityConversionFactor(ShooterConstants.kLeftEncoderVelocityFactor);

    leaderPIDController.setFeedbackDevice(leaderEncoder);
    leaderPIDController.setP(ShooterConstants.kLeftP);
    leaderPIDController.setI(ShooterConstants.kLeftI);
    leaderPIDController.setD(ShooterConstants.kLeftD);
    leaderPIDController.setIZone(ShooterConstants.kLeftIz);
    leaderPIDController.setFF(ShooterConstants.kLeftFF);
    leaderPIDController.setOutputRange(ShooterConstants.kLeftMinOutput,
        ShooterConstants.kLeftMaxOutput);

    leader.setInverted(ShooterConstants.kLeftMotorInverted);
    leader.setIdleMode(ShooterConstants.kLeftMotorIdleMode);
    leader.setSmartCurrentLimit(ShooterConstants.kLeftMotorCurrentLimit);

    follower.setIdleMode(ShooterConstants.kRightMotorIdleMode);
    follower.setSmartCurrentLimit(ShooterConstants.kRightMotorCurrentLimit);

    follower.follow(leader, true);

    leader.burnFlash();
    follower.burnFlash();

    stopShooter();
    setVelocitySP(0.0);

    System.out.println("----- Ending Shooter Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    sbVel.setDouble(getVelocity());
    sbVelSP.setDouble(getVelocitySP());
    sbPos.setDouble(getPosition());
    sbPosSP.setDouble(getPositionSP());
    sbTiltPos.setDouble(getTiltPos());
    sbTiltVolt.setDouble(getTiltVolt());
    sbTiltSP.setDouble(getTiltSP());
    sbNote.setBoolean(isNoteDetected());
  }

  public void stopShooter() {
    leader.stopMotor();
    tilt.stopMotor();
  }

  public boolean isNoteDetected() {
    return !noteDetect.get();
  }

  public void setTiltSP(double pos) {
    tiltSetPoint = MathUtil.clamp(pos, ShooterConstants.kMinTiltPos, ShooterConstants.kMaxTiltPos);
  }

  public double getTiltSP() {
    return tiltSetPoint;
  }

  public void setVelocitySP(double vel) {
    velSetPoint = MathUtil.clamp(vel, ShooterConstants.kMinShootRPM,
        ShooterConstants.kMaxShootRPM);
  }

  public double getVelocitySP() {
    return velSetPoint;
  }

  public void setPositionSP(double pos) {
    posSetPoint = pos;
  }

  public double getPositionSP() {
    return (posSetPoint);
  }

  public boolean atVelocity() {
    return ShooterConstants.kShootTollerance > Math.abs(getVelocitySP() - getVelocity());
  }

  public boolean atPosition() {
    return ShooterConstants.kShootTollerance > Math.abs(getPositionSP() - getPosition());
  }

  public boolean atTiltPos() {
    return ShooterConstants.kTiltTollerance > Math.abs(getTiltSP() - getTiltPos());
  }

  public void holdPosition(double pos) {
    setPositionSP(pos);
    leaderPIDController.setReference(getPositionSP(), CANSparkFlex.ControlType.kPosition);
  }

  public void holdVelocity(double vel) {
    setVelocitySP(vel);
    leaderPIDController.setReference(getVelocitySP(), CANSparkFlex.ControlType.kVelocity);
  }

  public void holdTilt(double pos) {
    setTiltSP(pos);
    tiltPIDController.setReference(getTiltSP() + ShooterConstants.kTiltPotAdj, CANSparkMax.ControlType.kPosition);
  }

  public void tiltTrackStick(double inc) {
    double pos = getTiltPos();
    pos += (inc * 0.01);
    holdTilt(pos);
  }

  public double getVelocity() {
    return leaderEncoder.getVelocity();
  }

  public double getPosition() {
    return leaderEncoder.getPosition();
  }

  public double getTiltPos() {
    return tiltEncoder.getPosition();
  }

  public double getTiltVolt() {
    return tiltEncoder.getVoltage();
  }

  public void moveToTrapPos() {
    stopShooter();
    double pos = getPosition() + 100.0;
    holdPosition(pos);
  }
}
