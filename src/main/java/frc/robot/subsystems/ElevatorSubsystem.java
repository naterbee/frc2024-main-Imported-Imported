package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.config.BaseConfig;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANDeviceID;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotMath.Elevator;
import frc.robot.Constants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ElevatorSubsystem extends SubsystemBase
{

  // This gearbox represents a gearbox containing 1 Neo
  // private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);

  // Standard classes for controlling our elevator
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kElevatorkS,
          ElevatorConstants.kElevatorkG,
          ElevatorConstants.kElevatorkV,
          ElevatorConstants.kElevatorkA);
  private final SparkMax                  m_motor      = new SparkMax(Constants.CANDeviceID.elevatorMotor1, MotorType.kBrushless);
  private final SparkMax            m_motorFollower = new SparkMax(Constants.CANDeviceID.elevatorMotor2, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
  private final RelativeEncoder           m_encoder    = m_motor.getEncoder();
  // private final SparkMaxSim               m_motorSim   = new SparkMaxSim(m_motor, m_elevatorGearbox);
  private final SparkMaxConfig elevatorConfig;

  

  // Simulation classes help us simulate what's going on, including gravity.
  // private final ElevatorSim m_elevatorSim =
  //     new ElevatorSim(
  //         m_elevatorGearbox,
  //         ElevatorConstants.kElevatorGearing,
  //         ElevatorConstants.kCarriageMass,
  //         ElevatorConstants.kElevatorDrumRadius,
  //         ElevatorConstants.kMinElevatorHeightMeters,
  //         ElevatorConstants.kMaxElevatorHeightMeters,
  //         true,
  //         0,
  //         0.01,
  //         0.0);

  // // Create a Mechanism2d visualization of the elevator
  // private final Mechanism2d         m_mech2d         = new Mechanism2d(20, 12);
  // private final MechanismRoot2d     m_mech2dRoot     = m_mech2d.getRoot("Elevator Root", 10, 0);
  // private final MechanismLigament2d m_elevatorMech2d =
  //     m_mech2dRoot.append(
  //         new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  /**
   * Subsystem constructor.
   */
  public ElevatorSubsystem()
  {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(40)
        .closedLoopRampRate(0.25)
        .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd)
          .outputRange(-1,1)
          .maxMotion
            .maxVelocity(Elevator.convertDistanceToRotations(Meters.of(0.25)).per(Second).in(RPM))
            .maxAcceleration(Elevator.convertDistanceToRotations(Meters.of(2)).per(Second).per(Second)
                                     .in(RPM.per(Second)));
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    elevatorConfig = new SparkMaxConfig();

    elevatorConfig.follow(CANDeviceID.elevatorMotor1, false);
    m_motorFollower.configure(elevatorConfig, null, null);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    // SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  /**
   * Advance the simulation.
   */
  public void simulationPeriodic()
  {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    // m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // // Next, we update it. The standard loop time is 20ms.
    // m_elevatorSim.update(0.020);

    // // Finally, we set our simulated encoder's readings and simulated battery voltage
    // m_motorSim.iterate(
    //     Elevator.convertDistanceToRotations(Meters.of(m_elevatorSim.getVelocityMetersPerSecond())).per(Second).in(RPM),
    //     RoboRioSim.getVInVoltage(),
    //     0.020);

    // // SimBattery estimates loaded battery voltages
    // RoboRioSim.setVInVoltage(
    //     BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  
  //  public static void manual (double velocity) {
  //   m_motor.setVoltage(m_feedforward.calculate(velocity));
  //  }

   public void manual (double speed) {
    m_motor.set(speed);
   }

   public void zeroEncoder () {
    m_encoder.setPosition(0);
   }

   public Command manualMove(double speed) {
    return runEnd(() -> { manual(speed); },
                  () -> { stop();});
  }


   public void reachGoal(double goal)
  {
    m_controller.setReference(goal, ControlType.kPosition);

    // m_controller.setReference(30, 
    //                           ControlType.kMAXMotionPositionControl,
    //                           ClosedLoopSlot.kSlot0,
    //                           m_feedforward.calculate(
    //                               Elevator.convertRotationsToDistance(Rotations.of(m_encoder.getVelocity())).per(Minute)
    //                                       .in(MetersPerSecond)));
   
      

}

  public void lowerElevator() {
    m_controller.setReference(m_encoder.getPosition()-0.5, ControlType.kPosition);
  }

  public void higherElevator() {
    m_controller.setReference(m_encoder.getPosition()+1, ControlType.kPosition);
  }


  /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getHeight()
  {
    // return Elevator.convertRotationsToDistance(Rotations.of(m_encoder.getPosition())).in(Meters);
   return m_encoder.getPosition();
  }

  /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in Meters
   * @param tolerance Tolerance in meters.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height,
                                             getHeight(),
                                             tolerance));
  }

  /**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setGoal(double goal)
  {
    return run(() -> reachGoal(goal));
  }

  public Command lower() {
    return run(() -> lowerElevator());
  }

  public Command higher() {
    return run(() -> higherElevator());
  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop()
  {
    m_motor.set(0.0);
  }

  /**
   * Update telemetry, including the mechanism visualization.
   */
  public void updateTelemetry()
  {
    // Update elevator visualization with position
   // m_elevatorMech2d.setLength(RobotBase.isSimulation() ? m_elevatorSim.getPositionMeters() : m_encoder.getPosition());
  }

  @Override
  public void periodic()
  {
    updateTelemetry();
    SmartDashboard.putNumber("elevator lead", m_motor.getAppliedOutput());
    SmartDashboard.putNumber("elevator follower", m_motorFollower.getAppliedOutput());
    
  }
}