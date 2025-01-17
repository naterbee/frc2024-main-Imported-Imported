// package frc.robot.subsystems;

// import static edu.wpi.first.units.MutableMeasure.mutable;
// import static edu.wpi.first.units.Units.Radians;
// import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.Rotations;
// import static edu.wpi.first.units.Units.RotationsPerSecond;
// import static edu.wpi.first.units.Units.Second;
// import static edu.wpi.first.units.Units.Volts;
// import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
// import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.REVPhysicsSim;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.sparkCANSparkBase.IdleMode;

// import edu.wpi.first.math.MathSharedStore;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.Measure;
// import edu.wpi.first.units.MutableMeasure;
// import edu.wpi.first.units.measure.Velocity;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.util.sendable.SendableBuilderImpl;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.CANDeviceID;
// import frc.robot.Constants.DIOPort;
// import frc.robot.util.CANSparkMaxSendable;

// // This mechanism uses
// //   NEO v1.1 brushless motor: https://www.revrobotics.com/rev-21-1650/ 
// //   REV Through Bore Encoder: https://www.revrobotics.com/rev-11-1271/ 
// //   Spark Max motor controller: https://www.revrobotics.com/rev-11-2158/

// // Arm theory (combine feeedback and feedforward controls):
// // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
// //
// // Is it the case that Trapezoidal motion profile is needed because feedforward
// // can only be calculated for a given angle, so need fine-grained progression of setpoints?
// // The article above hints at that, in
// // "accurately converge to the setpoint over time after a “jump” command"

// // Switching to spark's internal PID calculator might be better
// //  for reasons explained in https://www.chiefdelphi.com/t/spark-max-pid/340527/7 
// //  more precise control and  
// // https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control
// // and then we'd switch to ProfiledPIDSubsystem to TrapezoidProfileSubsystem
// // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armbotoffboard/subsystems/ArmSubsystem.java
// // and https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
// // Will need to configure setFeedbackDevice like it is done here
// // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Encoder%20Feedback%20Device/src/main/java/frc/robot/Robot.java
// //
// // Hardware setup for it: https://docs.revrobotics.com/sparkmax/operating-modes/using-encoders
// //
// // BUT: unclear if can also get absolute signal when the encoderis used in this mode.s

// // Sysid code is taken from
// // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/sysid/subsystems/Shooter.java
// // ProfiledPIDSubsysem implementation is from
// // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armbot/subsystems/ArmSubsystem.java

// // Don't use Rev's SmartMotion, explained in https://www.chiefdelphi.com/t/understanding-and-tuning-smart-motion-for-an-arm/426639/5
// // and acknowledged by Rev in a reply to the post.

// // All angular quantities without units are in radians.

// // Note on the method of motor control:
// //   SparkMax has different control modes (kCtrlType in https://docs.revrobotics.com/sparkmax/software-resources/configuration-parameters)
// //   and they cannot be mixed. Set/get is Duty Cycle and set/getVoltage is Voltage. Explanation here
// //     https://www.chiefdelphi.com/t/sparkmax-set-vs-setvoltage/415059
// //   set/getVoltage should be better for us because we need absolute power control when
// //   trying to move to an angle in presence of gravitational force. Also
// //   https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/combining-feedforward-feedback.html#using-feedforward-components-with-pid
// //      Since feedforward voltages are physically meaningful, it is best to use the setVoltage()
// //      method when applying them to motors to compensate for “voltage sag” from the battery."
// //
// //   (Velocity and Position are two other control modes and these seem to be not useful for a
// //   mechanism whose feedforward component is identified using SysId, which is all about voltage)


// // public class ArmSubsystem extends ProfiledPIDSubsystem {
// public class ArmSubsystem extends SubsystemBase {
//   private PIDController m_pidController;
//   private final CANSparkMaxSendable m_motorFollower;
//   private final CANSparkMaxSendable m_motor;
//   private final DutyCycleEncoder m_absEncoder;
//   private final Encoder m_relEncoder;
//   private final RelativeEncoder m_neoEncoder;
//   private final ArmFeedforward m_feedforward = new ArmFeedforward(
//           ArmConstants.kS.in(Volts), ArmConstants.kG.in(Volts),
//           ArmConstants.kV.in(VoltsPerRadianPerSecond),
//           ArmConstants.kA.in(VoltsPerRadianPerSecondSquared));

//   private final SysIdRoutine m_sysIdRoutine;

//   // Mutable holders for unit-safe values, persisted to avoid reallocation.
//   private final MutVoltage m_appliedVoltage = mutable(Volts.of(0));
//   private final MutAngle m_angle = mutable(Rotations.of(0));
//   private final MutAngularVelocity m_velocity = mutable(RotationsPerSecond.of(0));
//   private boolean m_enabled = false;

//   public ArmSubsystem() {
//     /* 
//     super(new ProfiledPIDController(
//             ArmConstants.kP,
//             ArmConstants.kI,
//             ArmConstants.kD,
//             new TrapezoidProfile.Constraints(
//                 ArmConstants.kMaxVelocity.in(RadiansPerSecond),
//                 ArmConstants.kMaxAcceleration.in(RadiansPerSecond.per(Second)))));
//             */
//     m_pidController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
//     m_pidController.setTolerance(Units.degreesToRadians(5));
//     // Start pointing up
//     setGoal(Units.degreesToRadians(90));

//     m_motor = new CANSparkMaxSendable(CANDeviceID.kArmMotors.id1(), MotorType.kBrushless);
//     m_motorFollower = new CANSparkMaxSendable(CANDeviceID.kArmMotors.id2(), MotorType.kBrushless);
//     m_motor.restoreFactoryDefaults();
//     m_motorFollower.restoreFactoryDefaults();
//     m_motor.setIdleMode(IdleMode.kBrake);
//     m_motorFollower.setIdleMode(IdleMode.kBrake);

//     m_motorFollower.isFollower(m_motor, true);
//     m_absEncoder = new DutyCycleEncoder(DIOPort.kDutyEncoder);
//     m_relEncoder = new Encoder(DIOPort.kQuadratureEncoderChannelA, DIOPort.kQuadratureEncoderChannelB);
//     // This returns the internal hall sensor whose counts per revolution is 42, very low.
//     // Probably not useful because the resolution is so poor.
//     m_neoEncoder = m_motor.getEncoder();

//     // See the explanation of CPR and PPR at
//     // https://docs.wpilib.org/en/stable/docs/hardware/sensors/encoders-hardware.html#quaderature-encoder-resolution
//     // For our Through Bore Encoder, Cycles per Revolution is 2048 per http://revrobotics.com/rev-11-1271/
//     final int kRevThoroughBoreEncoderPPR = 2048;
//     // 1 / PPR is how many rotations per pulse
//     m_relEncoder.setDistancePerPulse(Units.rotationsToRadians(1.0 / kRevThoroughBoreEncoderPPR));
//     // Similar, set absolute encoder units to be in radians
//     m_absEncoder.setDistancePerRotation(Units.rotationsToRadians(1));

//     PIDSendable pid_sendable = new PIDSendable();

//     // LiveWindow
//     addChild("motor", m_motor);
//     addChild("abs encoder", m_absEncoder);
//     addChild("rel encoder", m_relEncoder);
//     addChild("pid", pid_sendable);

//     // Regular SmartDashboard
//     SmartDashboard.putData("arm/sendable/subsystem", this);
//     SmartDashboard.putData("arm/sendable/encoder/abs", m_absEncoder);
//     SmartDashboard.putData("arm/sendable/encoder/rel", m_relEncoder);
//     SmartDashboard.putData("arm/sendable/motor", m_motor);
//     SmartDashboard.putData("arm/sendable/pid", pid_sendable);

//     m_sysIdRoutine = new SysIdRoutine(
//         // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
//         new SysIdRoutine.Config(),
//         new SysIdRoutine.Mechanism(
//             // Tell SysId how to plumb the driving voltage to the motor(s).
//             (Voltage volts) -> { m_motor.setVoltage(volts.in(Volts)); },
//             // Tell SysId how to record a frame of data for each motor on the mechanism
//             // being characterized.
//             log -> {
//               // Record a frame for the shooter motor.
//               log.motor("arm")
//                   .voltage(
//                       // Annoingly there is no getVoltage() method and get() cannot be used with
//                       // setVoltage();
//                       // see the discussion at
//                       // https://www.chiefdelphi.com/t/sysid-routine-not-properly-recording-motor-speed/455172
//                       m_appliedVoltage.mut_replace(
//                           // TODO: remove this the next line works
//                           // _motor.get() * RobotController.getBatteryVoltage(),
//                           m_motor.getBusVoltage() * m_motor.getAppliedOutput(),
//                           Volts))
//                   .angularPosition(m_angle.mut_replace(getMeasurement(), Radians))
//                   .angularVelocity(m_velocity.mut_replace(m_relEncoder.getRate(), RadiansPerSecond));
//             },
//             // Tell SysId to make generated commands require this subsystem, suffix test
//             // state in
//             // WPILog with this subsystem's name ("ArmSubsystem")
//             this));
//     addSysidCommandToDashboard(sysIdQuasistatic(SysIdRoutine.Direction.kForward).withName("fwd quas"));
//     addSysidCommandToDashboard(sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withName("back quas"));
//     addSysidCommandToDashboard(sysIdDynamic(SysIdRoutine.Direction.kForward).withName("fwd dynamic"));
//     addSysidCommandToDashboard(sysIdDynamic(SysIdRoutine.Direction.kReverse).withName("back dynamic"));
//   }

//   public boolean atSetpoint() {
//     return m_pidController.atSetpoint();
//   }

//   private class PIDSendable implements Sendable {
//     public void initSendable(SendableBuilder builder) {
//       builder.setSmartDashboardType("ProfiledPIDController");
//       builder.addDoubleProperty("p", m_pidController::getP, m_pidController::setP);
//       builder.addDoubleProperty("i", m_pidController::getI, m_pidController::setI);
//       builder.addDoubleProperty("d", m_pidController::getD, m_pidController::setD);
//       builder.addDoubleProperty(
//         "izone",
//         m_pidController::getIZone,
//         (double toSet) -> {
//           try {
//             m_pidController.setIZone(toSet);
//           } catch (IllegalArgumentException e) {
//             MathSharedStore.reportError("IZone must be a non-negative number!", e.getStackTrace());
//           }
//         });
//       builder.addDoubleProperty("goal", ArmSubsystem.this::getGoal, ArmSubsystem.this::setGoal);
//     }
//   }

//   public void simulationInit() {
//     REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNEO(1));
//     REVPhysicsSim.getInstance().addSparkMax(m_motorFollower, DCMotor.getNEO(1));
//   }

//   private void addSysidCommandToDashboard(Command cmd) {
//     SmartDashboard.putData("arm/sysid/" + cmd.getName(), cmd);
//   }

//   public void setGoal(double goal) {
//     m_pidController.setSetpoint(goal);
//   }
//   double getGoal() {
//     return m_pidController.getSetpoint();
//   }

//   public void enable() { m_enabled = true; }
//   public void disable() { m_enabled = false; }
//   public boolean isEnabled() { return m_enabled; }
 
//   /*
//   @Override
//   public void useOutput(double output, TrapezoidProfile.State setpoint) {
//     // Calculate the feedforward from the setpoint
//     double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
//     // Add the feedforward to the PID output to get the motor output
//     m_motor.setVoltage(output + feedforward);
//     SmartDashboard.putNumber("arm/output/output", output);
//     SmartDashboard.putNumber("arm/output/feedforward", feedforward);
//     SmartDashboard.putNumber("arm/output/position", setpoint.position);
//     SmartDashboard.putNumber("arm/output/velocity", setpoint.velocity);
//   }
//   */
//   void setVoltageWithPID() {
//     double voltage = 0;
//     if (m_enabled) {
//       double pid_voltage = m_pidController.calculate(getMeasurement());
//       double ff_voltage = m_feedforward.calculate(getMeasurement(), 0);
//       SmartDashboard.putNumber("arm/controller/pid_voltage", pid_voltage);
//       SmartDashboard.putNumber("arm/controller/ff_voltage", ff_voltage);
//       voltage = pid_voltage + ff_voltage;
//       double batVoltage = RobotController.getBatteryVoltage() * 0.8;
//       if (voltage > batVoltage) voltage = batVoltage;
//       if (voltage < -batVoltage) voltage = -batVoltage;
//     }
//     SmartDashboard.putNumber("arm/controller/voltage", voltage);
//     m_motor.setVoltage(voltage);
//   }

//   // @Override
//   public double getMeasurement() {
//     // getDistance returns radians because we set the appropriate setDistancePerRotation
//     // in the constructor.
//     return ((m_absEncoder.getDistance() + ArmConstants.kArmOffset.in(Radians)) % (Math.PI * 2));
//   }

//   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
//     return m_sysIdRoutine.quasistatic(direction).until(angleNotSafeSupplier(direction));
//   }

//   public Command sysIdDynamic(SysIdRoutine.Direction direction) {
//     return m_sysIdRoutine.dynamic(direction).until(angleNotSafeSupplier(direction));
//   }

//   public void stop() {
//     m_motor.setVoltage(0);

//   }

//   public void move(DoubleSupplier s) {
//     m_motor.setVoltage(12.0 * s.getAsDouble());
//     // m_motor.set(s.getAsDouble());
//   }

//   public Command unsafeMoveArm(DoubleSupplier speed) {
//     return runEnd(() -> { move(speed); },
//                   () -> { stop();}).withName("unsafeMoveArm");
//   }

//   public Command moveArm(DoubleSupplier speed) {
//     return unsafeMoveArm(speed).until(
//       angleNotSafeSupplier(speed)).withName("safeMoveArm");
//   }

//   private boolean angleNotSafe(boolean forward) {
//     double angle = getMeasurement();
//     if (forward) return angle > ArmConstants.kMaxAngleForward.in(Radians);
//     return angle < ArmConstants.kMaxAngleBackward.in(Radians);
//   }

//   private BooleanSupplier angleNotSafeSupplier(SysIdRoutine.Direction direction) {
//     return () -> angleNotSafe(direction == SysIdRoutine.Direction.kForward);
//   }

//   private BooleanSupplier angleNotSafeSupplier(DoubleSupplier speed) {
//     return () -> angleNotSafe(speed.getAsDouble() > 0);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     super.periodic();

//     setVoltageWithPID();

//     SmartDashboard.putBoolean("arm/output/enabled", isEnabled());
//     SmartDashboard.putNumber("arm/encoder/abs/raw", m_absEncoder.getDistance());
//     SmartDashboard.putNumber("arm/encoder/abs/adjusted", getMeasurement());
//     SmartDashboard.putNumber("arm/encoder/relative", m_relEncoder.getRate());
//     SmartDashboard.putNumber("arm/encoder/built-in", m_neoEncoder.getVelocity());
//     SmartDashboard.putNumber("arm/motor/duty_cycle", m_motor.get());
//     SmartDashboard.putNumber("arm/motor/bus_voltage", m_motor.getBusVoltage());
//     SmartDashboard.putNumber("arm/motor/applied_output", m_motor.getAppliedOutput());
//     SmartDashboard.putNumber("arm/motor/voltage_compensation", m_motor.getVoltageCompensationNominalVoltage());
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }
// }
