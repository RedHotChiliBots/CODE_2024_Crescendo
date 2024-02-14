// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.FeederConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Feeder extends SubsystemBase {
  /** Creates a new Hopper. */
  private final ShuffleboardTab feederTab = Shuffleboard.getTab("Feeder");

  private final CANSparkMax feeder = new CANSparkMax(CANIdConstants.kFeederCANId, MotorType.kBrushless);

  private final RelativeEncoder feederEncoder = feeder.getEncoder();
  private final SparkPIDController feederPIDController = feeder.getPIDController();

  private double setPoint = 0.0;

  public Feeder() {
    System.out.println("+++++ Starting Hopper Constructor +++++");

    feederEncoder.setPositionConversionFactor(FeederConstants.kFeederEncoderPositionFactor);
    feederEncoder.setVelocityConversionFactor(FeederConstants.kFeederEncoderVelocityFactor);

    feeder.setInverted(FeederConstants.kFeederMotorInverted);

    feederPIDController.setP(FeederConstants.kFeederP);
    feederPIDController.setI(FeederConstants.kFeederI);
    feederPIDController.setD(FeederConstants.kFeederD);
    feederPIDController.setFF(FeederConstants.kFeederFF);
    feederPIDController.setOutputRange(FeederConstants.kFeederMinOutput,
        FeederConstants.kFeederMaxOutput);

    feeder.setIdleMode(FeederConstants.kFeederMotorIdleMode);
    feeder.setSmartCurrentLimit(FeederConstants.kFeederMotorCurrentLimit);

    feeder.burnFlash();

    System.out.println("----- Ending Hopper Constructor -----");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setSetPoint(double s) {
    setPoint = s;
  }

  public double getSetPoint() {
    return (setPoint);
  }

  public void holdVel(double vel) {
    feederPIDController.setReference(vel, CANSparkMax.ControlType.kVelocity);
  }
}
