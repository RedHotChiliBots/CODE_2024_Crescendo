// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {
  /** Creates a new Hopper. */
  private final ShuffleboardTab feederTab = Shuffleboard.getTab("Feeder");
  private final GenericEntry sbVel = feederTab.addPersistent("Velocity", 0)
      .withWidget("Text View").withPosition(2, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbVelSP = feederTab.addPersistent("Velocity SP", 0)
      .withWidget("Text View").withPosition(4, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbPos = feederTab.addPersistent("Position", 0)
      .withWidget("Text View").withPosition(2, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbPosSP = feederTab.addPersistent("Position SP", 0)
      .withWidget("Text View").withPosition(4, 1).withSize(2, 1).getEntry();
  private final CANSparkMax feeder = new CANSparkMax(CANIdConstants.kFeederCANId, MotorType.kBrushless);

  private final RelativeEncoder feederEncoder = feeder.getEncoder();
  private final SparkPIDController feederPIDController = feeder.getPIDController();

  private double velSetPoint = 0.0;
  private double posSetPoint = 0.0;

  public Feeder() {
    System.out.println("+++++ Starting Hopper Constructor +++++");

    feeder.restoreFactoryDefaults();
    feeder.clearFaults();

    feederEncoder.setPositionConversionFactor(FeederConstants.kFeederEncoderPositionFactor);
    feederEncoder.setVelocityConversionFactor(FeederConstants.kFeederEncoderVelocityFactor);

    feederPIDController.setP(FeederConstants.kFeederP);
    feederPIDController.setI(FeederConstants.kFeederI);
    feederPIDController.setD(FeederConstants.kFeederD);
    feederPIDController.setFF(FeederConstants.kFeederFF);
    feederPIDController.setOutputRange(FeederConstants.kFeederMinOutput,
        FeederConstants.kFeederMaxOutput);

    feeder.setInverted(FeederConstants.kFeederMotorInverted);
    feeder.setIdleMode(FeederConstants.kFeederMotorIdleMode);
    feeder.setSmartCurrentLimit(FeederConstants.kFeederMotorCurrentLimit);

    feeder.burnFlash();

    feeder.stopMotor();
    setVelocitySP(0.0);
    
    System.out.println("----- Ending Hopper Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    sbVel.setDouble(getCurrVel());
    sbVelSP.setDouble(getVelocitySP());
    sbPos.setDouble(getCurrPos());
    sbPosSP.setDouble(getPositionSP());
  }

  public void stopFeeder() {
    feeder.stopMotor();
  }

  public void setVelocitySP(double vel) {
    velSetPoint = MathUtil.clamp(vel, FeederConstants.kMinFeederVel,
        FeederConstants.kMaxFeederVel);
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

  public void holdVelocity(double vel) {
    setVelocitySP(vel);
    feederPIDController.setReference(getVelocitySP(), CANSparkMax.ControlType.kVelocity);
  }

  public void holdPosition(double pos) {
    setPositionSP(pos);
    feederPIDController.setReference(getPositionSP(), CANSparkMax.ControlType.kPosition);
  }

  public double getCurrVel() {
    return feederEncoder.getVelocity();
  }

  public double getCurrPos() {
    return feederEncoder.getPosition();
  }
}
