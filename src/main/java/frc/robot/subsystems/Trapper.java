// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
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

  private final ShuffleboardTab trapperTab = Shuffleboard.getTab("Trapper");
  private final GenericEntry sbLiftPos = trapperTab.addPersistent("Lift Pos", 0)
      .withWidget("Text View").withPosition(2, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbLiftVolt = trapperTab.addPersistent("Lift Volt", 0)
      .withWidget("Text View").withPosition(6, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbLiftPosSP = trapperTab.addPersistent("Lift SP", 0)
      .withWidget("Text View").withPosition(4, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbTiltPos = trapperTab.addPersistent("Tilt Pos", 0)
      .withWidget("Text View").withPosition(2, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbTiltPosSP = trapperTab.addPersistent("Tilt SP", 0)
      .withWidget("Text View").withPosition(4, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbLeftClawPos = trapperTab.addPersistent("Left Claw Pos", 0)
      .withWidget("Text View").withPosition(2, 2).withSize(2, 1).getEntry();
  private final GenericEntry sbRightClawPos = trapperTab.addPersistent("RightClaw Pos", 0)
      .withWidget("Text View").withPosition(4, 2).withSize(2, 1).getEntry();
  private final GenericEntry sbClawPosSP = trapperTab.addPersistent("Claw SP", 0)
      .withWidget("Text View").withPosition(6, 2).withSize(2, 1).getEntry();

  private final Servo leftClaw = new Servo(PWMConstants.kLeftServoID);
  private final Servo rightClaw = new Servo(PWMConstants.kRightServoID);

  private final CANSparkMax lift = new CANSparkMax(CANIdConstants.kLiftTrapCANId, MotorType.kBrushless);
  private final CANSparkMax tilt = new CANSparkMax(CANIdConstants.kTiltTrapCANId, MotorType.kBrushless);

  private final SparkAnalogSensor liftEncoder = lift.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
  private final SparkPIDController liftPIDController = lift.getPIDController();

  private final SparkAbsoluteEncoder tiltEncoder = tilt.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController tiltPIDController = tilt.getPIDController();

  private double liftSetPoint = 0.0;
  private double tiltSetPoint = 0.0;
  private double clawSetPoint = 0.0;

  public Trapper() {
    System.out.println("+++++ Starting Trapper Constructor +++++");

    lift.restoreFactoryDefaults();
    lift.clearFaults();
    tilt.restoreFactoryDefaults();
    tilt.clearFaults();

    liftEncoder.setPositionConversionFactor(TrapperConstants.kLiftEncoderPositionFactor);
    tiltEncoder.setPositionConversionFactor(TrapperConstants.kTiltEncoderPositionFactor);

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

    setLiftSP(TrapperConstants.kMinLiftLen);
    setTiltSP(TrapperConstants.kMinTiltDeg);

    leftClaw.set(TrapperConstants.kGripOpen);
    rightClaw.set(TrapperConstants.kGripOpen);

    System.out.println("----- Ending Trapper Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sbLiftPos.setDouble(getLiftPosition());
    sbLiftVolt.setDouble(getLiftVoltage());
    sbLiftPosSP.setDouble(getLiftSP());
    sbTiltPos.setDouble(getTiltPosition());
    sbTiltPosSP.setDouble(getTiltSP());
    sbLeftClawPos.setDouble(getLeftClawPos());
    sbRightClawPos.setDouble(getRightClawPos());
    sbClawPosSP.setDouble(getClawSP());
  }

  public void stopTrapper() {
    lift.stopMotor();
    tilt.stopMotor();
  }

  public void setLiftSP(double pos) {
    liftSetPoint = MathUtil.clamp(pos, TrapperConstants.kMinLiftLen + TrapperConstants.kPotMin,
        TrapperConstants.kMaxLiftLen + TrapperConstants.kPotMin);
  }

  public void holdLift(double pos) {
    setLiftSP(pos);
    liftPIDController.setReference(getLiftSP(), CANSparkMax.ControlType.kPosition);
  }

  public void setTiltSP(double deg) {
    tiltSetPoint = Math.toRadians(MathUtil.clamp(deg, TrapperConstants.kMinTiltDeg,
        TrapperConstants.kMaxTiltDeg));
  }

  public void holdTilt(double deg) {
    setTiltSP(deg);
    tiltPIDController.setReference(getTiltSP(), CANSparkMax.ControlType.kPosition);
  }

  public void setClawSP(double pos) {
    clawSetPoint = MathUtil.clamp(pos, TrapperConstants.kMinClawDeg,
        TrapperConstants.kMaxClawDeg);
  }

  public void setClaw(double pos) {
    setClawSP(pos);
    leftClaw.set(getClawSP());
    rightClaw.set(getClawSP());
  }

  public double getClawSP() {
    return clawSetPoint;
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
    return Math.toDegrees(tiltSetPoint);
  }

  public double getLeftClawPos() {
    return leftClaw.get();
  }

  public double getRightClawPos() {
    return rightClaw.get();
  }
}
