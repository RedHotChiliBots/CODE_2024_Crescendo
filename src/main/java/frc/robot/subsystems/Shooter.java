// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.DigitalIOConstants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

  private final CANSparkMax tilt = new CANSparkMax(CANIdConstants.kTiltShooterCANID, MotorType.kBrushless);
  private final CANSparkFlex leader = new CANSparkFlex(CANIdConstants.kLeftShooterCANID, MotorType.kBrushless);
  private final CANSparkFlex follower = new CANSparkFlex(CANIdConstants.kRightShooterCANID, MotorType.kBrushless);

  private final RelativeEncoder tiltEncoder = tilt.getEncoder();
  private final SparkPIDController tiltPIDController = tilt.getPIDController();
  private final RelativeEncoder leaderEncoder = leader.getEncoder();
  private final SparkPIDController leaderPIDController = leader.getPIDController();

  private final DigitalInput noteDetect = new DigitalInput(DigitalIOConstants.kShooterNoteDetect);

  private double tiltSetPoint = 0.0;
  private double velSetPoint = 0.0;
  private double posSetPoint = 0.0;

  public Shooter() {
    System.out.println("+++++ Starting Shooter Constructor +++++");

    follower.follow(leader);

    tiltEncoder.setPositionConversionFactor(ShooterConstants.kTiltEncoderPositionFactor);
    tiltEncoder.setVelocityConversionFactor(ShooterConstants.kTiltEncoderVelocityFactor);

    tilt.setInverted(ShooterConstants.kTiltMotorInverted);

    tiltPIDController.setP(ShooterConstants.kTiltP);
    tiltPIDController.setI(ShooterConstants.kTiltI);
    tiltPIDController.setD(ShooterConstants.kTiltD);
    tiltPIDController.setFF(ShooterConstants.kTiltFF);
    tiltPIDController.setOutputRange(ShooterConstants.kTiltMinOutput,
        ShooterConstants.kTiltMaxOutput);

    tilt.setIdleMode(ShooterConstants.kTiltMotorIdleMode);
    tilt.setSmartCurrentLimit(ShooterConstants.kTiltMotorCurrentLimit);

    tilt.burnFlash();

    leaderEncoder.setPositionConversionFactor(ShooterConstants.kLeftEncoderPositionFactor);
    leaderEncoder.setVelocityConversionFactor(ShooterConstants.kLeftEncoderVelocityFactor);

    leader.setInverted(ShooterConstants.kLeftMotorInverted);

    leaderPIDController.setP(ShooterConstants.kLeftP);
    leaderPIDController.setI(ShooterConstants.kLeftI);
    leaderPIDController.setD(ShooterConstants.kLeftD);
    leaderPIDController.setFF(ShooterConstants.kLeftFF);
    leaderPIDController.setOutputRange(ShooterConstants.kLeftMinOutput,
        ShooterConstants.kLeftMaxOutput);

    leader.setIdleMode(ShooterConstants.kLeftMotorIdleMode);
    leader.setSmartCurrentLimit(ShooterConstants.kLeftMotorCurrentLimit);

    leader.burnFlash();

    follower.setInverted(ShooterConstants.kRightMotorInverted);

    follower.setIdleMode(ShooterConstants.kRightMotorIdleMode);
    follower.setSmartCurrentLimit(ShooterConstants.kRightMotorCurrentLimit);

    follower.burnFlash();

    System.out.println("----- Ending Shooter Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isNoteDetected() {
    return noteDetect.get();
  }
  
  public void setTilt(double deg) {
    tiltSetPoint = deg;
  }

  public double getTilt() {
    return (tiltSetPoint);
  }

  public void setVelocity(double vel) {
    tiltSetPoint = vel;
  }

  public double getVelocity() {
    return (velSetPoint);
  }

  public void setPosition(double pos) {
    posSetPoint = pos;
  }

  public double getPosition() {
    return (posSetPoint);
  }

  public void holdTilt(double deg) {
    leaderPIDController.setReference(deg, CANSparkMax.ControlType.kPosition);
  }

  public void holdVel(double vel) {
    leaderPIDController.setReference(vel, CANSparkMax.ControlType.kVelocity);
  }

  public void holdPos(double pos) {
    leaderPIDController.setReference(pos, CANSparkMax.ControlType.kPosition);
  }

  public double getCurrPos() {
    return leaderEncoder.getPosition();
  }
}
