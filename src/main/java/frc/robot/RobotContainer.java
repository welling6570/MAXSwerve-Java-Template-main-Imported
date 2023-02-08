// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Trajectories;
import java.lang.reflect.Array;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final SwerveDriveOdometry​ m_swerveDriveOdometry = new SwerveDriveOdometry​(kDriveKinematics);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  //private final SuppliedValueWidget statewidget = new SuppliedValueWidget(null, null, null, null, null);
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  
  private SwerveControllerCommand midSwerveControllerCommand;
  private SwerveControllerCommand shortSwerveControllerCommand;
  private SwerveControllerCommand longSwerveControllerCommand;
  private SwerveControllerCommand gigaMidSwerveControllerCommand;
 
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    UsbCamera fisheye = CameraServer.startAutomaticCapture(0);
    fisheye.setResolution(160, 120);
    fisheye.setFPS(15);
    m_chooser.setDefaultOption("offtarmac", m_midengage);
    Trajectories.generateTrajectories();
    generateSwerveCommands();
    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
    m_chooser.addOption("lessofftarmac", m_long);
    m_chooser.addOption("lessofftarmac", m_shortking);
    private final List<Trajectory.State> robotPositionList = Trajectories.midTrajectory.getStates();
    SmartDashboard.putNumber("example", robotPositionList[0]);
   
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                //MathUtil.applyDeadband(-m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY()), 0.06),
                -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY()),0.02)),
                //MathUtil.applyDeadband(-m_driverController.getLeftX() * Math.abs(m_driverController.getLeftX()), 0.06),
                -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX() * Math.abs(m_driverController.getLeftX()), 0.02)),
                //MathUtil.applyDeadband(-m_driverController.getRightX() * Math.abs(m_driverController.getRightX()), 0.06),
                -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driverController.getRightX()* Math.abs(m_driverController.getRightX()), 0.02)),
                (m_driverController.getLeftBumper() ? false : true)),
            m_robotDrive));
  }
  private final SequentialCommandGroup m_midengage = new SequentialCommandGroup(

  );
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(3.1415)));
   
    

    // Run path following command, then stop at the end.
    return midSwerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  private void generateSwerveCommands() {
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);    
    
    midSwerveControllerCommand = new SwerveControllerCommand(
        Trajectories.midTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    shortSwerveControllerCommand = new SwerveControllerCommand(
        Trajectories.shortTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);    
            
    longSwerveControllerCommand = new SwerveControllerCommand(
        Trajectories.longTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    gigaMidSwerveControllerCommand = new SwerveControllerCommand(
        Trajectories.gigaMidTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);
  }

}
