// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.TunableNumber;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;
// import frc.robot.subsystems.OuttakeSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

// @SuppressWarnings("unused")
// @SuppressWarnings("unused")
public class RobotContainer
{
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  private final CommandXboxController m_driverXbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_secondaryDriverXbox =
      new CommandXboxController(OperatorConstants.kSecondaryDriverControllerPort);
      
   private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
   private final OuttakeSubsystem m_outtake = new OuttakeSubsystem();

  TunableNumber m_angleP = new TunableNumber("Swerve/PID/ModuleAngle/P", SwerveParser.pidfPropertiesJson.angle.p);
  TunableNumber m_angleD = new TunableNumber("Swerve/PID/ModuleAngle/D", SwerveParser.pidfPropertiesJson.angle.d);
  TunableNumber m_driveP = new TunableNumber("Swerve/PID/ModuleDrive/P", SwerveParser.pidfPropertiesJson.drive.p);
  TunableNumber m_driveD = new TunableNumber("Swerve/PID/ModuleDrive/D", SwerveParser.pidfPropertiesJson.drive.d);

  private SendableChooser<Command> m_autoChooser = null;


  public RobotContainer() {
    Command driveFieldOrientedAnglularVelocity = m_drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(m_driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(m_driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(m_driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        m_driverXbox.getHID()::getAButtonPressed)
        .withName("driveFieldOriented");

    Command driveRobotOriented = m_drivebase.driveCommandRobotRelative(
        () -> -MathUtil.applyDeadband(m_driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(m_driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(m_driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND))
        .withName("driveRobotOriented");
    addCommandToDashboard(driveRobotOriented);

    Command zeroGyro = m_drivebase.runOnce(() -> m_drivebase.zeroGyro()).withName("zeroGyro");
    addCommandToDashboard(zeroGyro);

    Command zeroEncoder = m_elevator.runOnce(() -> m_elevator.zeroEncoder()).withName("zeroElevator");
    addCommandToDashboard(zeroEncoder);

    Command resetOdometrytoAllianceZero = m_drivebase.runOnce(
        () -> m_drivebase.resetOdometry(m_drivebase.invertIfFieldFlipped(new Pose2d(0, 0, new Rotation2d()))))
        .withName("resetOdometrytoAllianceZero");
    addCommandToDashboard(resetOdometrytoAllianceZero);

    Command addFakeVisionReading = m_drivebase.runOnce(() -> m_drivebase.addFakeVisionReading())
        .withName("addFakeVisionReading");
    addCommandToDashboard(addFakeVisionReading);

    Command testMotors = m_drivebase.run(() -> {
      SwerveDriveTest.powerAngleMotorsDutyCycle(m_drivebase.swerveDrive, m_driverXbox.getLeftX());
      SwerveDriveTest.powerDriveMotorsDutyCycle(m_drivebase.swerveDrive, m_driverXbox.getLeftY());
    }).withName("testMotors");
    addCommandToDashboard(testMotors);

    Command testAngleMotors = m_drivebase.run(() -> {
      SwerveDriveTest.angleModules(m_drivebase.swerveDrive, 
        // Divided by 2 because want to go -180 to 180, not full circle
        Rotation2d.fromRotations(m_driverXbox.getLeftX() / 2));
    }).withName("testAngleMotors");
    addCommandToDashboard(testAngleMotors);

    TunableNumber angle = new TunableNumber("testAngle", 90);
    Command testSetAngle = m_drivebase.runEnd(
        () -> SwerveDriveTest.angleModules(m_drivebase.swerveDrive, Rotation2d.fromDegrees(angle.get())),
        () -> SwerveDriveTest.angleModules(m_drivebase.swerveDrive, Rotation2d.fromDegrees(0)))
        .withName("testSetAngle");
    addCommandToDashboard(testSetAngle);

    NamedCommands.registerCommand("elevatorL2", m_elevator.setGoal(35));
    NamedCommands.registerCommand("outtake!", m_outtake.outtake(-.1).withTimeout(0.75));
    NamedCommands.registerCommand("elevatorBase", m_elevator.setGoal(6));

     // NamedCommands.registerCommand("outtake", m_outtake.outtake(2));
    /*
     * Command testDriveToPose = drivebase.runOnce(
     * () -> drivebase.resetOdometry(new Pose2d(0, 0, new Rotation2d()))).andThen(
     * drivebase.driveToPose(
     * new Pose2d(new Translation2d(1, 1), Rotation2d.fromDegrees(0))))
     * .withName("testDriveToPose");
     */
    // Command testDriveToPose = m_drivebase.driveToRelativePose(
    //   new Pose2d(new Translation2d(Meters.of(0), Meters.of(0.1)),
    //                                new Rotation2d(Degrees.of(0))))
    //     /* .asProxy() */.withName("testDriveToPose");
    // addCommandToDashboard(testDriveToPose);

    m_drivebase.setDefaultCommand(
        
        driveFieldOrientedAnglularVelocity);
    m_driverXbox.leftBumper().whileTrue(driveRobotOriented);

   
  // ELEVATOR COMMANDS
    // outake alignment / L1
    m_secondaryDriverXbox.x().whileTrue(m_elevator.setGoal(24.6));
    // L2
    m_secondaryDriverXbox.y().whileTrue(m_elevator.setGoal(35));
    // base
    m_secondaryDriverXbox.a().whileTrue(m_elevator.setGoal(6));
    // lower
    m_secondaryDriverXbox.leftBumper().whileTrue(m_elevator.lower());
    // higher
    m_secondaryDriverXbox.rightBumper().whileTrue(m_elevator.higher());


    // outtake / backwards outtake
    m_secondaryDriverXbox.rightTrigger().whileTrue(m_outtake.outtake(-0.1));
    m_secondaryDriverXbox.leftTrigger().whileTrue(m_outtake.outtake(0.075));

    // limelight align
    m_secondaryDriverXbox.b().whileTrue(m_drivebase.limelightDriveCommand());

  


    SmartDashboard.putData(CommandScheduler.getInstance());

     m_autoChooser = AutoBuilder.buildAutoChooser();
     SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  private void addCommandToDashboard(Command cmd) {
    SmartDashboard.putData("cmd/" + cmd.getName(), cmd);
  }

  public void simulationInit() {
   // m_arm.simulationInit();
  }

  public void robotPeriodic() {
    SmartDashboard.putNumber("joystick/left-X", m_driverXbox.getLeftX());
    SmartDashboard.putNumber("joystick/left-Y", m_driverXbox.getLeftY());
    SmartDashboard.putNumber("joystick/right-X", m_driverXbox.getRightX());
    SmartDashboard.putNumber("joystick/right-Y", m_driverXbox.getRightY());

    SmartDashboard.putNumber("pose/x", m_drivebase.getPose().getX());
    SmartDashboard.putNumber("pose/y", m_drivebase.getPose().getY());
    SmartDashboard.putNumber("pose/z", m_drivebase.getPose().getRotation().getDegrees());

    SmartDashboard.putNumber("elevatorHeight", m_elevator.getHeight());

    SmartDashboard.putString("alliance", m_drivebase.isFieldFlipped() ? "RED" : "BLUE");

    if (m_angleD.hasChanged() || m_angleP.hasChanged()) {
      for (SwerveModule module : m_drivebase.swerveDrive.getModules()) {
        module.getAngleMotor().configurePIDF(new PIDFConfig(m_angleP.get(), m_angleD.get()));
      }
    }
    if (m_driveD.hasChanged() || m_driveP.hasChanged()) {
      for (SwerveModule module : m_drivebase.swerveDrive.getModules()) {
        module.getDriveMotor().configurePIDF(new PIDFConfig(m_driveP.get(), m_driveD.get()));
      }
    }
  }

  public Command getAutonomousCommand() {
    
     return m_autoChooser.getSelected();

    // return m_drivebase.driveAtSpeed(5, 0, 0, false).withTimeout(0.5);

 }

  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }
}