// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CANSparkMaxSendable;
import frc.robot.Constants.CANDeviceID;
import com.revrobotics.spark.SparkMax;

public class OuttakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkMax m_motor;
  private final SparkMax m_motorFollower;
  private final SparkMaxConfig config;

  public OuttakeSubsystem() {
    m_motor = new SparkMax(CANDeviceID.kOuttakeMotor.id1(), MotorType.kBrushless);
    m_motorFollower = new SparkMax(CANDeviceID.kOuttakeMotor.id2(), MotorType.kBrushless);
    config = new SparkMaxConfig();

    config.follow(CANDeviceID.kOuttakeMotor.id1());
    m_motorFollower.configure(config, null, null);
    
  }


  public void intake (double speed) {
    m_motor.set(speed);
  }

  public void stop () {
    m_motor.set(0);
  }

  public Command outtake(double speed) {
    return runEnd(() -> { intake(speed); },
                  () -> { stop();});
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
