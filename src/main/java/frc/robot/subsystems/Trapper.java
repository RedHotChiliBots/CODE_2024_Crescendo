// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.PWMConstants;
import frc.robot.Constants.TrapperConstants;

public class Trapper extends SubsystemBase {
  /** Creates a new Trapper. */
  private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");
  private final ShuffleboardTab trapperTab = Shuffleboard.getTab("Trapper");
  private final GenericEntry sbLiftPos = compTab.addPersistent("Lift Pos", 0)
      .withWidget("Text View").withPosition(4, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbLiftVolt = compTab.addPersistent("Lift Volt", 0)
      .withWidget("Text View").withPosition(4, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbLiftPosSP = compTab.addPersistent("Lift SP", 0)
      .withWidget("Text View").withPosition(4, 2).withSize(2, 1).getEntry();
  private final GenericEntry sbTiltPos = compTab.addPersistent("Tilt Pos", 0)
      .withWidget("Text View").withPosition(4, 3).withSize(2, 1).getEntry();
  private final GenericEntry sbTiltPosSP = compTab.addPersistent("Tilt SP", 0)
      .withWidget("Text View").withPosition(4, 4).withSize(2, 1).getEntry();
  private final GenericEntry sbLeftClawPos = compTab.addPersistent("Left Claw Pos", 0)
      .withWidget("Text View").withPosition(4, 5).withSize(2, 1).getEntry();
  private final GenericEntry sbRightClawPos = compTab.addPersistent("RightClaw Pos", 0)
      .withWidget("Text View").withPosition(4, 6).withSize(2, 1).getEntry();

  private final Servo topClaw = new Servo(PWMConstants.kTopServoID);
  private final Servo botClaw = new Servo(PWMConstants.kBotServoID);

  private final CANSparkMax lift = new CANSparkMax(CANIdConstants.kLiftTrapCANId, MotorType.kBrushless);
  private final CANSparkMax tilt = new CANSparkMax(CANIdConstants.kTiltTrapCANId, MotorType.kBrushless);

  private final SparkAnalogSensor liftEncoder = lift.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
  private final SparkPIDController liftPIDController = lift.getPIDController();

  private final SparkAbsoluteEncoder tiltEncoder = tilt.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController tiltPIDController = tilt.getPIDController();

  private double liftSetPoint = getLiftPosition();
  private double tiltSetPoint = getTiltPosition();
  private CLAW clawSetPoint = CLAW.CLOSE;

  public enum CLAW {
    OPEN,
    CLOSE,
    NA
  }

  public CLAW clawPos = CLAW.NA;

  public Trapper() {
    System.out.println("+++++ Starting Trapper Constructor +++++");

    lift.restoreFactoryDefaults();
    lift.clearFaults();
    tilt.restoreFactoryDefaults();
    tilt.clearFaults();

    // CAN Status frames
    lift.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    lift.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    lift.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    lift.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);
    lift.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    lift.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 0);
    lift.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 0);

    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0);
    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 0);
    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200);
    tilt.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);

    liftEncoder.setPositionConversionFactor(TrapperConstants.kLiftEncoderPositionFactor);
    tiltEncoder.setPositionConversionFactor(TrapperConstants.kTiltEncoderPositionFactor);
    tiltEncoder.setInverted(TrapperConstants.kTiltEncoderInverted);
    tiltEncoder.setZeroOffset(TrapperConstants.kTiltZeroOffset);

    liftPIDController.setFeedbackDevice(liftEncoder);
    liftPIDController.setP(TrapperConstants.kLiftP);
    liftPIDController.setI(TrapperConstants.kLiftI);
    liftPIDController.setD(TrapperConstants.kLiftD);
    liftPIDController.setFF(TrapperConstants.kLiftFF);
    liftPIDController.setOutputRange(TrapperConstants.kLiftMinOutput,
        TrapperConstants.kLiftMaxOutput);

    lift.setInverted(TrapperConstants.kLiftMotorInverted);
    lift.setIdleMode(TrapperConstants.kLiftMotorIdleMode);
    lift.setSmartCurrentLimit(TrapperConstants.kLiftMotorCurrentLimit);

    lift.burnFlash();

    tiltPIDController.setFeedbackDevice(tiltEncoder);
    tiltPIDController.setP(TrapperConstants.kTiltP);
    tiltPIDController.setI(TrapperConstants.kTiltI);
    tiltPIDController.setD(TrapperConstants.kTiltD);
    tiltPIDController.setFF(TrapperConstants.kTiltFF);
    tiltPIDController.setOutputRange(TrapperConstants.kTiltMinOutput,
        TrapperConstants.kTiltMaxOutput);

    tilt.setInverted(TrapperConstants.kTiltMotorInverted);
    tilt.setIdleMode(TrapperConstants.kTiltMotorIdleMode);
    tilt.setSmartCurrentLimit(TrapperConstants.kTiltMotorCurrentLimit);

    tilt.burnFlash();

    lift.stopMotor();
    tilt.stopMotor();

    setTiltSP(TrapperConstants.kInitTiltDeg);
    setLiftSP(TrapperConstants.kMinLiftLen);

    holdClaw(CLAW.OPEN);

    System.out.println("----- Ending Trapper Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sbLiftPos.setDouble(getLiftPosition() - TrapperConstants.kLiftPotAdj);
    sbLiftVolt.setDouble(getLiftVoltage());
    sbLiftPosSP.setDouble(getLiftSP());
    sbTiltPos.setDouble(getTiltPosition());
    sbTiltPosSP.setDouble(getTiltSP());
    sbLeftClawPos.setDouble(getLeftClawPos());
    sbRightClawPos.setDouble(getRightClawPos());
  }

  public void stopTrapper() {
    lift.stopMotor();
    tilt.stopMotor();
  }

  public boolean atLiftSP() {
    return Math.abs(getLiftPosition() - getLiftSP()) < TrapperConstants.kLiftTollerance;
  }

  public boolean atTiltSP() {
    return Math.abs(getTiltPosition() - getTiltSP()) < TrapperConstants.kTiltTollerance;
  }

  public void setLiftSP(double pos) {
    liftSetPoint = MathUtil.clamp(pos, TrapperConstants.kMinLiftLen,
        TrapperConstants.kMaxLiftLen);
  }

  public void holdLift(double pos) {
    setLiftSP(pos);
    liftPIDController.setReference(getLiftSP() + TrapperConstants.kLiftPotAdj, CANSparkMax.ControlType.kPosition);
  }

  public void setTiltSP(double deg) {
    tiltSetPoint = MathUtil.clamp(deg, TrapperConstants.kMinTiltDeg, TrapperConstants.kMaxTiltDeg);
  }

  public void holdTilt(double deg) {
//    setTiltSP(deg);
    tiltPIDController.setReference(Math.toRadians(getTiltSP()), CANSparkMax.ControlType.kPosition);
  }

  public void holdClaw(CLAW clawPos) {
    switch (clawPos) {
      case OPEN:
        topClaw.setAngle(TrapperConstants.kTopClawOpen);
        botClaw.setAngle(TrapperConstants.kBotClawOpen);
        break;

      case CLOSE:
        topClaw.setAngle(TrapperConstants.kTopClawClose);
        botClaw.setAngle(TrapperConstants.kBotClawClose);
        break;

      case NA:
        DriverStation.reportError("Invalid value for CLAW", true);
        break;

      default:
        DriverStation.reportError("Unknown value for CLAW", true);
    }
  }

  public double getLiftPosition() {
    return liftEncoder.getPosition();
  }

  public double getLiftVoltage() {
    return liftEncoder.getVoltage();
  }

  public double getLiftSP() {
    return liftSetPoint;
  }

  public double getTiltPosition() {
    return tiltEncoder.getPosition();
  }

  public double getTiltSP() {
    return tiltSetPoint;
  }

  public double getLeftClawPos() {
    return topClaw.getAngle();
  }

  public double getRightClawPos() {
    return botClaw.getAngle();
  }
}
