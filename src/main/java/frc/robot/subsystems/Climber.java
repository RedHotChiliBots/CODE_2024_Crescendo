// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  /** Creates a new Climber. */
  private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
  private final GenericEntry sbLeftPos = climberTab.addPersistent("Left Pos", 0)
      .withWidget("Text View").withPosition(2, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbLeftPosSP = climberTab.addPersistent("Left Pos SP", 0)
      .withWidget("Text View").withPosition(4, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbRightPos = climberTab.addPersistent("Right Pos", 0)
      .withWidget("Text View").withPosition(2, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbRightPosSP = climberTab.addPersistent("Right Pos SP", 0)
      .withWidget("Text View").withPosition(4, 1).withSize(2, 1).getEntry();

  private final CANSparkMax leftLeader = new CANSparkMax(CANIdConstants.kLeft1CANId, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(CANIdConstants.kLeft2CANId, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(CANIdConstants.kRight1CANId, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(CANIdConstants.kRight2CANId, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
  // private final SparkAnalogSensor leftEncoder =
  // leftLeader.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
  // private final SparkAnalogSensor rightEncoder =
  // rightLeader.getAnalog(SparkAnalogSensor.Mode.kAbsolute);

  private final SparkPIDController leftPIDController = leftLeader.getPIDController();
  private final SparkPIDController rightPIDController = rightLeader.getPIDController();

  private double posSetPoint = ClimberConstants.kMinClimbPos;

  private final Relay brake = new Relay(0);

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

    // CAN Status frames
    leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);
    leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 0);
    leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 0);

    rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);
    rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 0);
    rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 0);

    leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
    leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);
    leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 0);
    leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 0);

    rightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    rightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    rightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
    rightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);
    rightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    rightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 0);
    rightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 0);

    leftEncoder.setPositionConversionFactor(ClimberConstants.kClimberEncoderPositionFactor);
    rightEncoder.setPositionConversionFactor(ClimberConstants.kClimberEncoderPositionFactor);

    // leftEncoder.setInverted(true);
    // rightEncoder.setInverted(true);

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
    // leftFollower.setInverted(ClimberConstants.kLeft2Inverted);
    leftFollower.setIdleMode(ClimberConstants.kLeft2IdleMode);
    leftLeader.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);
    leftFollower.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);

    rightLeader.setInverted(ClimberConstants.kRight1Inverted);
    rightLeader.setIdleMode(ClimberConstants.kRight1IdleMode);
    // rightFollower.setInverted(ClimberConstants.kRight2Inverted);
    rightFollower.setIdleMode(ClimberConstants.kRight2IdleMode);
    rightLeader.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);
    rightFollower.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);

    leftFollower.follow(leftLeader, false);
    rightFollower.follow(rightLeader, false);

    leftLeader.burnFlash();
    leftFollower.burnFlash();
    rightLeader.burnFlash();
    rightFollower.burnFlash();

    stopClimber();
    initClimber();
    setClimbSP(ClimberConstants.kMinClimbPos);

    brake.set(Relay.Value.kOff);

    System.out.println("----- Ending Climber Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    sbLeftPos.setDouble(getLeftPosition());
    sbLeftPosSP.setDouble(getPositionSP());
    sbRightPos.setDouble(getRightPosition());
    sbRightPosSP.setDouble(getPositionSP());

    SmartDashboard.putNumber("Leader Power", leftLeader.get());
    SmartDashboard.putNumber("Follower Power", leftFollower.get());
  }

  public void initClimber() {
    leftEncoder.setPosition(ClimberConstants.kMinClimbPos);
    rightEncoder.setPosition(ClimberConstants.kMinClimbPos);
  }

  public void stopClimber() {
    leftLeader.stopMotor();
    rightLeader.stopMotor();
  }

  public void setBrakeOn() {
    brake.set(Relay.Value.kOn);
  }

  public void setBrakeOff() {
    brake.set(Relay.Value.kOff);
  }

  public void setClimbSP(double pos) {
    posSetPoint = MathUtil.clamp(pos, ClimberConstants.kMinClimbPos,
        ClimberConstants.kMaxClimbPos);
  }

  public void holdClimbPos(double pos) {
    setClimbSP(pos);
    leftPIDController.setReference(getPositionSP(), CANSparkMax.ControlType.kPosition);
    rightPIDController.setReference(getPositionSP(), CANSparkMax.ControlType.kPosition);
  }

  public boolean atClimberSP() {
    return ClimberConstants.kClimberTollerance > Math.abs(getLeftPosition() - getPositionSP());
  }

  public void climb(double spd) {
    leftLeader.set(spd);
    rightLeader.set(spd);
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
}
