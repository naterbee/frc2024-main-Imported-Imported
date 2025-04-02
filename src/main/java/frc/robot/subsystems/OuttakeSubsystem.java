// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CANSparkMaxSendable;
import frc.robot.Constants.CANDeviceID;
import com.revrobotics.spark.SparkMax;

public class OuttakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkFlex m_motor;
  private final SparkFlex m_motorFollower;
  private final SparkFlexConfig config;

  public OuttakeSubsystem() {
    m_motor = new SparkFlex(CANDeviceID.kOuttakeMotor1, MotorType.kBrushless);
    m_motorFollower = new SparkFlex(CANDeviceID.kOuttakeMotor2, MotorType.kBrushless);
    config = new SparkFlexConfig();

    config.follow(CANDeviceID.kOuttakeMotor1, true);
    // m_motorFollower.configure(config, null, null);
    
  }

// Motor Speed Controls //
  public void intake () {
    m_motor.set(-0.15);
    m_motorFollower.set(0.03);
  }

  public void intakeStraight () {
    m_motor.set(-0.1);
    m_motorFollower.set(0.1);
  }

  public void stop () {
    m_motor.set(0);
  }

  public Command outtake() {
    return runEnd(() -> { intake(); },
                  () -> { stop();});
  }

  public Command outtakeStraight() {
    return runEnd(() -> { intakeStraight(); },
                  () -> { stop();});
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
