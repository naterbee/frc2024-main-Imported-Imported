// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDeviceID;
import frc.robot.util.CANSparkMaxSendable;

public class IntakeSubsystem extends SubsystemBase {
 private final CANSparkMaxSendable m_motor;

  public IntakeSubsystem() {
     m_motor = new CANSparkMaxSendable(CANDeviceID.kIntakeMotor, MotorType.kBrushless);
    // m_motor.restoreFactoryDefaults();
     m_motor.clearFaults();
    addChild("motor", m_motor);
  }

  public void intake(double speed) {
    m_motor.set(speed);
  }

  public void stop() {
    m_motor.set(0);
  }

  public Command intakeCommand(double speed) {
    return runEnd(() -> { intake(speed); },
                  () -> { stop();});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
