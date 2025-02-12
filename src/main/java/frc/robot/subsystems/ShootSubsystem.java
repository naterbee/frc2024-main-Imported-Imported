// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CANSparkMaxSendable;
import frc.robot.Constants;
import frc.robot.Constants.CANDeviceID;

public class ShootSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMaxSendable m_motor;

  public ShootSubsystem() {
    m_motor = new CANSparkMaxSendable(CANDeviceID.kIntakeMotor, MotorType.kBrushless);
  }


  public void shoot (double speed) {
    m_motor.set(speed);
  }

  public void stop () {
    m_motor.set(0);
  }

  public Command shootCommand(double speed) {
    return runEnd(() -> { shoot(speed); },
                  () -> { stop();});
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
