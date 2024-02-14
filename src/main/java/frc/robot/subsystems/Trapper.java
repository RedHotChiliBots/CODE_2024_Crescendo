// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.PWMConstants;
import frc.robot.Constants.TrapperConstants;

public class Trapper extends SubsystemBase {
  /** Creates a new Trapper. */

  private final ShuffleboardTab trapperTab = Shuffleboard.getTab("Trapper");

  private final Servo leftGrip = new Servo(PWMConstants.kLeftServoID);
  private final Servo rightGrip = new Servo(PWMConstants.kRightServoID);

  private final CANSparkMax liftTrap = new CANSparkMax(CANIdConstants.kLiftTrapCANId, MotorType.kBrushless);
  private final CANSparkMax tiltTrap = new CANSparkMax(CANIdConstants.kTiltTrapCANId, MotorType.kBrushless);

  private final RelativeEncoder liftEncoder = liftTrap.getEncoder();
  private final SparkPIDController liftPIDController = liftTrap.getPIDController();
  private final RelativeEncoder tiltEncoder = tiltTrap.getEncoder();
  private final SparkPIDController tiltPIDController = tiltTrap.getPIDController();

  private final AbsoluteEncoder TrapTiltABSEncoder = tiltTrap.getAbsoluteEncoder(Type.kDutyCycle);

  private double tiltSetPoint = 0.0;
  private double lenSetPoint = 0.0;

  public Trapper() {
    System.out.println("+++++ Starting Trapper Constructor +++++");

    liftEncoder.setPositionConversionFactor(TrapperConstants.kLiftEncoderPositionFactor);
    liftEncoder.setVelocityConversionFactor(TrapperConstants.kLiftEncoderVelocityFactor);

    liftTrap.setInverted(TrapperConstants.kLiftMotorInverted);

    liftPIDController.setP(TrapperConstants.kLiftP);
    liftPIDController.setI(TrapperConstants.kLiftI);
    liftPIDController.setD(TrapperConstants.kLiftD);
    liftPIDController.setFF(TrapperConstants.kLiftFF);
    liftPIDController.setOutputRange(TrapperConstants.kLiftMinOutput,
        TrapperConstants.kLiftMaxOutput);

    liftTrap.setIdleMode(TrapperConstants.kLiftMotorIdleMode);
    liftTrap.setSmartCurrentLimit(TrapperConstants.kLiftMotorCurrentLimit);

    liftTrap.burnFlash();

    tiltEncoder.setPositionConversionFactor(TrapperConstants.kTiltEncoderPositionFactor);
    tiltEncoder.setVelocityConversionFactor(TrapperConstants.kTiltEncoderVelocityFactor);

    tiltTrap.setInverted(TrapperConstants.kTiltMotorInverted);

    tiltPIDController.setP(TrapperConstants.kTiltP);
    tiltPIDController.setI(TrapperConstants.kTiltI);
    tiltPIDController.setD(TrapperConstants.kTiltD);
    tiltPIDController.setFF(TrapperConstants.kTiltFF);
    tiltPIDController.setOutputRange(TrapperConstants.kTiltMinOutput,
        TrapperConstants.kTiltMaxOutput);

    tiltTrap.setIdleMode(TrapperConstants.kTiltMotorIdleMode);
    tiltTrap.setSmartCurrentLimit(TrapperConstants.kTiltMotorCurrentLimit);

    tiltTrap.burnFlash();

    liftTrap.stopMotor();
    tiltTrap.stopMotor();
    leftGrip.set(TrapperConstants.kGripOpen);
    rightGrip.set(TrapperConstants.kGripOpen);

    System.out.println("----- Ending Trapper Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTilt(double deg) {
    tiltSetPoint = deg;
  }

  public double getTilt() {
    return (tiltSetPoint);
  }

  public void setLength(double len) {
    lenSetPoint = len;
  }

  public double getLength() {
    return (lenSetPoint);
  }

  public void holdTilt(double deg) {
    tiltPIDController.setReference(deg, CANSparkMax.ControlType.kPosition);
  }

  public void holdLen(double len) {
    liftPIDController.setReference(len, CANSparkMax.ControlType.kPosition);
  }
}
