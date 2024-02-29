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
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
  private final GenericEntry sbVel = intakeTab.addPersistent("Velocity", 0)
      .withWidget("Text View").withPosition(2, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbVelSP = intakeTab.addPersistent("Velocity SP", 0)
      .withWidget("Text View").withPosition(4, 0).withSize(2, 1).getEntry();
  private final GenericEntry sbPos = intakeTab.addPersistent("Position", 0)
      .withWidget("Text View").withPosition(2, 1).withSize(2, 1).getEntry();
  private final GenericEntry sbPosSP = intakeTab.addPersistent("Position SP", 0)
      .withWidget("Text View").withPosition(4, 1).withSize(2, 1).getEntry();

  private final CANSparkMax intake = new CANSparkMax(CANIdConstants.kIntakeCANId, MotorType.kBrushless);

  private final RelativeEncoder intakeEncoder = intake.getEncoder();
  private final SparkPIDController intakePIDController = intake.getPIDController();

  private double velSetPoint = 0.0;
  private double posSetPoint = 0.0;

  public Intake() {
    System.out.println("+++++ Starting Intake Constructor +++++");

    intake.restoreFactoryDefaults();
    intake.clearFaults();

    intakeEncoder.setPositionConversionFactor(IntakeConstants.kIntakeEncoderPositionFactor);
    intakeEncoder.setVelocityConversionFactor(IntakeConstants.kIntakeEncoderVelocityFactor);

    // intakePIDController.setFeedbackDevice(intakeEncoder);
    intakePIDController.setP(IntakeConstants.kIntakeP);
    intakePIDController.setI(IntakeConstants.kIntakeI);
    intakePIDController.setD(IntakeConstants.kIntakeD);
    intakePIDController.setFF(IntakeConstants.kIntakeFF);
    intakePIDController.setOutputRange(IntakeConstants.kIntakeMinOutput,
        IntakeConstants.kIntakeMaxOutput);

    intake.setInverted(IntakeConstants.kIntakeMotorInverted);
    intake.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
    intake.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

    intake.burnFlash();

    intake.stopMotor();
    setVelocitySP(0.0);

    System.out.println("----- Ending Intake Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sbVel.setDouble(getVelocity());
    sbVelSP.setDouble(getVelocitySP());
    sbPos.setDouble(getPosition());
    sbPosSP.setDouble(getPositionSP());
  }

  public void stopIntake() {
    intake.stopMotor();
  }

  public void setVelocitySP(double vel) {
    velSetPoint = MathUtil.clamp(vel, IntakeConstants.kMinIntakeVel,
        IntakeConstants.kMaxIntakeVel);
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

  public void setVelocity(double vel) {
    setVelocitySP(vel);
    intake.set(IntakeConstants.kIntakeVelocity);
    // intakePIDController.setReference(getVelocitySP(),
    // CANSparkMax.ControlType.kVelocity);
  }

  public void holdPosition(double pos) {
    setPositionSP(pos);
    intakePIDController.setReference(getPositionSP(), CANSparkMax.ControlType.kPosition);
  }

  public double getVelocity() {
    return intakeEncoder.getVelocity();
  }

  public double getPosition() {
    return intakeEncoder.getPosition();
  }
}
