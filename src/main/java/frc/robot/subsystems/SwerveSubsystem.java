// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.GeometryUtil;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.spark.SparkMax;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Robot;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.util.CANSparkMaxSendableAdapter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

// Notes for if we ever decide to SysId this swerve:
// - the math is here https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#the-permanent-magnet-dc-motor-feedforward-equation
// - the default feedforward controller in YAGSL is configured without Ks term,
//   which I find  strange, as Ks seems to be the most useful part
//   (see here https://github.com/BroncBotz3481/YAGSL-Lib/blob/2024/src/main/java/swervelib/parser/SwerveParser.java#L116
//    and SwerveMath.createDriveFeedforward that it calls)
//   - kV term is calculated as optimalVoltage / maxSpeed, which I do not understand,
//     should it not be a cruising voltage? "how much voltage is needed to hold (or “cruise”) at a given constant velocity",
//     according to the wpilib article referenced above. But also probably does not
//     matter because we do not need to hold constant speed anywhere
//     - maxSpeed is provided in the construction code below,
//     - optimalVoltage is from PhysicalPropertiesJson.swervelib.parser.json, 12V by
//       default. Can be overridden in physicalproperties.json.
// - thus, we might want to set our own feedforward, by calling SwerveDrive.replaceSwerveModuleFeedforward,
//   but be careful about other functions that reset it, SwerveDrive.setMaximumSpeed at the moment is the only one
// - the current SwerveSubsystem.sysIdDriveMotorCommand is too high-level, it does all
//   four routines in a single call, but we migth want to do it by hand for better
//   safety control (so that the robot does not ram into anything)
public class SwerveSubsystem extends SubsystemBase
{
  /**
   * Swerve drive object.
   */
  public final SwerveDrive swerveDrive;
  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    // These values came from
    // https://www.swervedrivespecialties.com/collections/kits/products/mk4i-swerve-module
    // we have L1 gearing ratio on the drive motor both motors.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(150.0/7);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
      Units.inchesToMeters(4), 8.14);

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    // sending lots of data to driver station
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(
        SwerveConstants.kMaxSpeed.in(MetersPerSecond), angleConversionFactor, driveConversionFactor);
      // Alternative method if conversion factors are supplied via JSON file
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);      
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.

    // setupPathPlanner();

    // 0 to 3. front left -> front right -> back left -> back right (from SverveModule.moduleNumber documentation)
    addLiveWindowModule("FL", 0);
    addLiveWindowModule("FR", 1);
    addLiveWindowModule("BL", 2);
    addLiveWindowModule("BR", 3);

    // Example of how to change a single motor's PID config
    // swerveDrive.getModules()[idx].configuration.anglePIDF = new PIDFConfig(0.1, 0, 0);

    SmartDashboard.putData("swerve/subsystem", this);
  }

  private void addLiveWindowModule(String name, int idx) {
    SwerveModule module = swerveDrive.getModules()[idx];

    /* addChild(name + " angle motor", new CANSparkMaxSendableAdapter(
      (CANSparkMax) module.getAngleMotor().getMotor()));
    addChild(name + " drive motor", new CANSparkMaxSendableAdapter(
      (CANSparkMax) module.getDriveMotor().getMotor()));
    addChild(name + " encoder",
      (Sendable) module.getAbsoluteEncoder().getAbsoluteEncoder()); */
      addChild(name + " angle motor", new CANSparkMaxSendableAdapter(
      (SparkMax) module.getAngleMotor().getMotor()));
    addChild(name + " drive motor", new CANSparkMaxSendableAdapter(
      (SparkMax) module.getDriveMotor().getMotor()));
    addChild(name + " encoder",
      (Sendable) module.getAbsoluteEncoder().getAbsoluteEncoder());
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg, 
                                  controllerCfg, 
                                  SwerveConstants.kMaxSpeed.in(MetersPerSecond),
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                  Rotation2d.fromDegrees(0)));
  }

  public boolean isFieldFlipped() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }
  public Pose2d invertIfFieldFlipped(Pose2d pose) {
    if (isFieldFlipped()) return FlippingUtil.flipFieldPose(pose);
    return pose;
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
 
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (ChassisSpeeds, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  ChassisSpeeds,
                  swerveDrive.kinematics.toSwerveModuleStates(ChassisSpeeds),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(ChassisSpeeds);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              PathPlannerConstants.kTranslationPID,
              PathPlannerConstants.kAnglePID
             
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  // public void setupPathPlanner()
  // { 

  //   AutoBuilder.configureHolonomic(
  //       this::getPose, // Robot pose supplier
  //       this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
  //       this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  //       this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
  //       new HolonomicPathFollowerConfig( 
  //         // HolonomicPathFollowerConfig, this should likely live in your Constants class
  //         PathPlannerConstants.kTranslationPID, // Translation PID constants
  //         PathPlannerConstants.kAnglePID, // Rotation PID constants
  //         PathPlannerConstants.kMaxSpeed.in(MetersPerSecond), // Max module speed, in m/s
  //         // Drive base radius in meters. Distance from robot center to furthest module.
  //         swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
  //         new ReplanningConfig() // Default path replanning config. See the API for the options here
  //       ),
  //       // Boolean supplier that controls when the path will be mirrored for the red alliance
  //       // This will flip the path being followed to the red side of the field.
  //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  //       () -> isFieldFlipped(),
  //       this // Reference to this subsystem to set requirements
  //   );
  // }

  /**
   * Aim the robot at the target returned by PhotonVision.
   *
   * @param camera {@link PhotonCamera} to communicate with.
   * @return A {@link Command} which will run the alignment.
   */
  // public Command aimAtTarget(PhotonCamera camera)
  // {
  //   return run(() -> {
  //     PhotonPipelineResult result = camera.getLatestResult();
  //     if (result.hasTargets())
  //     {
  //       drive(getTargetSpeeds(0,
  //                             0,
  //                             Rotation2d.fromDegrees(result.getBestTarget()
  //                                                          .getYaw()))); // Not sure if this will work, more math may be required.
  //     }
  //   });
  // }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose)
  {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
       edu.wpi.first.units.Units.MetersPerSecond.of(0)
    );
  }

  public Command driveToRelativePose(Pose2d pose) {
    return new WrapperCommand(driveToPose(pose)) {
      @Override
      public void initialize() {
        resetOdometry(new Pose2d(0, 0, new Rotation2d()));
        super.initialize();
      }
    };
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommandDirectAngle(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
      double xHeading = Math.pow(headingX.getAsDouble(), 3); // Smooth controll out
      double yHeading = Math.pow(headingY.getAsDouble(), 3); // Smooth controll out
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      xHeading, yHeading,
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  public Command driveCommandRobotRelative(DoubleSupplier x, DoubleSupplier y,
                                           DoubleSupplier z) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(x.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          y.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        z.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        // Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                        false,
                        false);
    });    
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                                                                      translationY.getAsDouble(),
                                                                      rotation.getAsDouble() * Math.PI,
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX,
                              BooleanSupplier slow)
  {
    return run(() -> {
      int multiplier = isFieldFlipped() ? -1 : 1;
      if (slow.getAsBoolean()) {
        multiplier *= 0.75;
      }
      // Make the robot move
      swerveDrive.drive(new Translation2d(
        multiplier * Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity(),
        multiplier * Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity()),
        multiplier * Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
        true,
        false);
    });
  }

  public Command driveAtSpeed(double x, double y, double z, boolean fieldRelative) {
    return run(() -> swerveDrive.drive(new Translation2d(x, y), z, fieldRelative, false));
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  @Override
  public void periodic()
  {
  }

  @Override
  public void simulationPeriodic()
  {
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        SwerveConstants.kMaxSpeed.in(MetersPerSecond));
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        SwerveConstants.kMaxSpeed.in(MetersPerSecond));
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }
}