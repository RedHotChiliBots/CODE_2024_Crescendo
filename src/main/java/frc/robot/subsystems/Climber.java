// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  /** Creates a new Climber. */
  private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
  private final GenericEntry sbLeftPos = climberTab.addPersistent("Left Pos", 0)
      .withWidget("Text View").withPosition(2, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbLeftVolt = climberTab.addPersistent("Left Volt", 0)
      .withWidget("Text View").withPosition(6, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbLeftPosSP = climberTab.addPersistent("Left Pos SP", 0)
      .withWidget("Text View").withPosition(4, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbRightPos = climberTab.addPersistent("Right Pos", 0)
      .withWidget("Text View").withPosition(2, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbRightVolt = climberTab.addPersistent("Right Volt", 0)
      .withWidget("Text View").withPosition(6, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbRightPosSP = climberTab.addPersistent("Right Pos SP", 0)
      .withWidget("Text View").withPosition(4, 1).withSize(2, 1).getEntry();

  private final CANSparkMax leftLeader = new CANSparkMax(CANIdConstants.kLeft1CANId, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(CANIdConstants.kLeft2CANId, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(CANIdConstants.kRight1CANId, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(CANIdConstants.kRight2CANId, MotorType.kBrushless);

  private final SparkAnalogSensor leftEncoder = leftLeader.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
  private final SparkAnalogSensor rightEncoder = rightLeader.getAnalog(SparkAnalogSensor.Mode.kAbsolute);

  private final SparkPIDController leftPIDController = leftLeader.getPIDController();
  private final SparkPIDController rightPIDController = rightLeader.getPIDController();

  private double posSetPoint = ClimberConstants.kMinClimbPos;

  public Climber() {
    System.out.println("+++++ Starting Climber Constructor +++++");

    leftLeader.restoreFactoryDefaults();
    leftLeader.clearFaults();
    leftFollower.restoreFactoryDefaults();
    leftFollower.clearFaults();
    rightLeader.restoreFactoryDefaults();
    rightLeader.clearFaults();
    rightFollower.restoreFactoryDefaults();
    rightFollower.clearFaults();

    leftEncoder.setPositionConversionFactor(ClimberConstants.kClimberEncoderPositionFactor);
    rightEncoder.setPositionConversionFactor(ClimberConstants.kClimberEncoderPositionFactor);

    leftPIDController.setFeedbackDevice(leftEncoder);
    leftPIDController.setP(ClimberConstants.kP);
    leftPIDController.setI(ClimberConstants.kI);
    leftPIDController.setD(ClimberConstants.kD);
    leftPIDController.setFF(ClimberConstants.kFF);
    leftPIDController.setOutputRange(ClimberConstants.kMinOutput,
        ClimberConstants.kMaxOutput);

    rightPIDController.setFeedbackDevice(rightEncoder);
    rightPIDController.setP(ClimberConstants.kP);
    rightPIDController.setI(ClimberConstants.kI);
    rightPIDController.setD(ClimberConstants.kD);
    rightPIDController.setFF(ClimberConstants.kFF);
    rightPIDController.setOutputRange(ClimberConstants.kMinOutput,
        ClimberConstants.kMaxOutput);

    leftLeader.setInverted(ClimberConstants.kLeft1Inverted);
    leftLeader.setIdleMode(ClimberConstants.kLeft1IdleMode);
    leftFollower.setIdleMode(ClimberConstants.kLeft2IdleMode);
    leftLeader.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);
    leftFollower.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);

    rightLeader.setInverted(ClimberConstants.kRight1Inverted);
    rightLeader.setIdleMode(ClimberConstants.kRight1IdleMode);
    rightFollower.setIdleMode(ClimberConstants.kRight2IdleMode);
    rightLeader.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);
    rightFollower.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);

    leftFollower.follow(leftLeader, true);
    rightFollower.follow(rightLeader, true);

    leftLeader.burnFlash();
    leftFollower.burnFlash();
    rightLeader.burnFlash();
    rightFollower.burnFlash();

    stopClimber();
    setClimbSP(ClimberConstants.kMinClimbPos);

    System.out.println("----- Ending Climber Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    sbLeftPos.setDouble(getLeftPosition());
    sbLeftVolt.setDouble(getLeftVoltage());
    sbLeftPosSP.setDouble(getPositionSP());
    sbRightPos.setDouble(getRightPosition());
    sbRightVolt.setDouble(getRightVoltage());
    sbRightPosSP.setDouble(getPositionSP());
  }

  public void stopClimber() {
    leftLeader.stopMotor();
    rightLeader.stopMotor();
  }

  public void setClimbSP(double pos) {
    posSetPoint = MathUtil.clamp(pos, ClimberConstants.kMinClimbPos,
        ClimberConstants.kMaxClimbPos);
  }

  public void holdClimbPos(double pos) {
    setClimbSP(pos);
    leftPIDController.setReference(getPositionSP(), CANSparkMax.ControlType.kPosition);
//    rightPIDController.setReference(getPositionSP(), CANSparkMax.ControlType.kPosition);
  }

  public double getPositionSP() {
    return posSetPoint;
  }

  public double getLeftPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightPosition() {
    return rightEncoder.getPosition();
  }

  public double getLeftVoltage() {
    return leftEncoder.getVoltage();
  }

  public double getRightVoltage() {
    return rightEncoder.getVoltage();
  }
}
