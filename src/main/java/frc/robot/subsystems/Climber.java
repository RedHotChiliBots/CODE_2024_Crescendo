// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.AnalogConstants;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Climber extends SubsystemBase {

  /** Creates a new Climber. */
  private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");

  private final CANSparkMax leftLeader = new CANSparkMax(CANIdConstants.kLeft1CANId, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(CANIdConstants.kLeft2CANId, MotorType.kBrushless);
  private final CANSparkMax rightLeader = new CANSparkMax(CANIdConstants.kRight1CANId, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(CANIdConstants.kRight2CANId, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final SparkPIDController leftPIDController = leftLeader.getPIDController();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
  private final SparkPIDController rightPIDController = rightLeader.getPIDController();

  private final AnalogPotentiometer leftPot = new AnalogPotentiometer(AnalogConstants.kClimberLeftPot, 180, 0);
  private final AnalogPotentiometer rightPot = new AnalogPotentiometer(AnalogConstants.kClimberRightPot, 180, 0);

  private double setPoint = 0.0;

  public Climber() {
    System.out.println("+++++ Starting Climber Constructor +++++");

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftEncoder.setPositionConversionFactor(ClimberConstants.kClimberEncoderPositionFactor);
    leftEncoder.setVelocityConversionFactor(ClimberConstants.kClimberEncoderVelocityFactor);

    leftLeader.setInverted(ClimberConstants.kLeft1Inverted);

    leftPIDController.setP(ClimberConstants.kP);
    leftPIDController.setI(ClimberConstants.kI);
    leftPIDController.setD(ClimberConstants.kD);
    leftPIDController.setFF(ClimberConstants.kFF);
    leftPIDController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

    leftLeader.setIdleMode(ClimberConstants.kLeft1IdleMode);
    leftLeader.setSmartCurrentLimit(ClimberConstants.kLeftCurrentLimit);

    leftLeader.burnFlash();

    leftFollower.setInverted(ClimberConstants.kLeft2Inverted);
    leftFollower.setIdleMode(ClimberConstants.kLeft2IdleMode);

    leftFollower.burnFlash();

    rightEncoder.setPositionConversionFactor(ClimberConstants.kClimberEncoderPositionFactor);
    rightEncoder.setVelocityConversionFactor(ClimberConstants.kClimberEncoderVelocityFactor);

    rightLeader.setInverted(ClimberConstants.kRight1Inverted);

    rightPIDController.setP(ClimberConstants.kP);
    rightPIDController.setI(ClimberConstants.kI);
    rightPIDController.setD(ClimberConstants.kD);
    rightPIDController.setFF(ClimberConstants.kFF);
    rightPIDController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

    rightLeader.setIdleMode(ClimberConstants.kRight1IdleMode);

    rightLeader.burnFlash();

    rightFollower.setInverted(ClimberConstants.kRight2Inverted);
    rightFollower.setIdleMode(ClimberConstants.kRight2IdleMode);

    rightFollower.burnFlash();

    System.out.println("----- Ending Climber Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSetPoint(double s) {
    setPoint = s;
  }

  public double getSetPoint() {
    return(setPoint);
  }

  public void setPos(double pos) {
    leftPIDController.setReference(pos, CANSparkMax.ControlType.kPosition);
    rightPIDController.setReference(pos, CANSparkMax.ControlType.kPosition);
  }
}
