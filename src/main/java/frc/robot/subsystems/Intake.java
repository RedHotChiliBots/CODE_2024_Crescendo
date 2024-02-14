// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

  private final CANSparkMax intake = new CANSparkMax(CANIdConstants.kIntakeCANId, MotorType.kBrushless);

  private final RelativeEncoder intakeEncoder = intake.getEncoder();
  private final SparkPIDController intakePIDController = intake.getPIDController();

  private double setPoint = 0.0;

  public Intake() {
    System.out.println("+++++ Starting Intake Constructor +++++");

    intakeEncoder.setPositionConversionFactor(IntakeConstants.kIntakeEncoderPositionFactor);
    intakeEncoder.setVelocityConversionFactor(IntakeConstants.kIntakeEncoderVelocityFactor);

    intake.setInverted(IntakeConstants.kIntakeMotorInverted);

    intakePIDController.setP(IntakeConstants.kIntakeP);
    intakePIDController.setI(IntakeConstants.kIntakeI);
    intakePIDController.setD(IntakeConstants.kIntakeD);
    intakePIDController.setFF(IntakeConstants.kIntakeFF);
    intakePIDController.setOutputRange(IntakeConstants.kIntakeMinOutput,
        IntakeConstants.kIntakeMaxOutput);

    intake.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
    intake.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

    intake.burnFlash();

    System.out.println("----- Ending Intake Constructor -----");
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
    intakePIDController.setReference(vel, CANSparkMax.ControlType.kVelocity);
  }
}
