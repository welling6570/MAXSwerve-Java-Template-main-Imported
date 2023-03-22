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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Trajectories;
import frc.robot.Commands.ArmManipulation;
import frc.robot.Commands.IntakeManipulation;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
    //private final SmartDashboard m_Dashboard;
    //private final Shuffleboard m_Shuffleboard;

  //private final SwerveDriveOdometry​ m_swerveDriveOdometry = new SwerveDriveOdometry​(kDriveKinematics);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  //private final SuppliedValueWidget statewidget = new SuppliedValueWidget(null, null, null, null, null);
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController XBShooter = new XboxController(OIConstants.kArmControllerPort);

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  
  private SwerveControllerCommand bluemidSwerveControllerCommand;
  private SwerveControllerCommand redmidSwerveControllerCommand;
  private SwerveControllerCommand shortSwerveControllerCommand;
  private SwerveControllerCommand noArmSwerveControllerCommand;
  private SwerveControllerCommand longSwerveControllerCommand;
  private SwerveControllerCommand gigaMidSwerveControllerCommand;
 
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // UsbCamera fisheye = CameraServer.startAutomaticCapture(0);
    // fisheye.setResolution(160, 120);
    // fisheye.setFPS(15);
   // m_chooser.setDefaultOption("mid auto", m_midengage);
    Trajectories.generateTrajectories();
    generateSwerveCommands();
    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
    m_chooser.setDefaultOption("blue mid", 
    new RunCommand(() -> m_arm.Angxtend(ArmConstants.releaseHook,ArmConstants.JeremyRennerAuto)).until(m_arm::getLift) 
    .andThen(new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorY,ArmConstants.JeremyRennerAuto)).until(m_arm::getReach))
    .andThen(new RunCommand(() -> m_intake.jeremyRennerHug()).withTimeout(ArmConstants.autonomousTimeOut)
    .andThen(new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorStow,ArmConstants.JeremyRennerAuto)).until(m_arm::reachReturned))
    .andThen(new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorStow,ArmConstants.JeremyRennerStow))
      .alongWith(bluemidSwerveControllerCommand
    .andThen(() -> m_robotDrive.drive(0, 0, 0, false, 0)))))
    );
    m_chooser.addOption("sides", 
    new RunCommand(() -> m_arm.Angxtend(ArmConstants.releaseHook,ArmConstants.JeremyRennerAuto)).until(m_arm::getLift) 
    .andThen(new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorY,ArmConstants.JeremyRennerAuto)).until(m_arm::getReach))
    .andThen(new RunCommand(() -> m_intake.jeremyRennerHug()).withTimeout(ArmConstants.autonomousTimeOut)
    .andThen(new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorStow,ArmConstants.JeremyRennerAuto)).until(m_arm::reachReturned))
    .andThen(new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorStow,ArmConstants.JeremyRennerStow))
      .alongWith(shortSwerveControllerCommand
    .andThen(() -> m_robotDrive.drive(0, 0, 0, false, 0)))))
    );
    m_chooser.addOption("red mid", 
    new RunCommand(() -> m_arm.Angxtend(ArmConstants.releaseHook,ArmConstants.JeremyRennerAuto)).until(m_arm::getLift) 
    .andThen(new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorY,ArmConstants.JeremyRennerAuto)).until(m_arm::getReach))
    .andThen(new RunCommand(() -> m_intake.jeremyRennerHug()).withTimeout(ArmConstants.autonomousTimeOut)
    .andThen(new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorStow,ArmConstants.JeremyRennerAuto)).until(m_arm::reachReturned))
    .andThen(new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorStow,ArmConstants.JeremyRennerStow))
      .alongWith(redmidSwerveControllerCommand
    .andThen(() -> m_robotDrive.drive(0, 0, 0, false, 0)))))
    );

    m_chooser.addOption("no arm", 
      new RunCommand(() -> noArmSwerveControllerCommand
      .andThen(() -> m_robotDrive.drive(0, 0, 0, false, 0))));

    //m_chooser.addOption("short auto", m_shortking);
    //private final List<Trajectory.State> robotPositionList = Trajectories.midTrajectory.getStates();
    

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                //MathUtil.applyDeadband(-m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY()), 0.06),
                -m_xspeedLimiter.calculate(MathUtil.applyDeadband(Math.pow(m_driverController.getLeftY(),3),0.02)),
                //MathUtil.applyDeadband(-m_driverController.getLeftX() * Math.abs(m_driverController.getLeftX()), 0.06),
                -m_yspeedLimiter.calculate(MathUtil.applyDeadband(Math.pow(m_driverController.getLeftX(), 3) , 0.02)),
                //MathUtil.applyDeadband(-m_driverController.getRightX() * Math.abs(m_driverController.getRightX()), 0.06),
                -m_rotLimiter.calculate(MathUtil.applyDeadband(Math.pow(m_driverController.getRightX(),3), 0.02)),
                (m_driverController.getLeftBumper() ? false : true),
                (m_driverController.getRightTriggerAxis())),
            m_robotDrive));
    m_intake.setDefaultCommand(
       new RunCommand(
        () -> m_intake.jeremyRennerVibeCheck(XBShooter.getLeftTriggerAxis(), XBShooter.getRightTriggerAxis()), m_intake
    ));
    m_arm.setDefaultCommand(
       new RunCommand(
        () -> m_arm.getPositions(), m_arm
    ));
  }

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
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    // new JoystickButton(XBShooter, Button.kA.value).onTrue( new RunCommand(() -> m_arm.Angxtend(50,0)));
    // new JoystickButton(XBShooter, Button.kA.value).onFalse( new RunCommand(() -> m_arm.Angxtend(50,0)));
    new JoystickButton(XBShooter, Button.kA.value)
      .whileTrue( new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorA, ArmConstants.JeremyRennerA)));
    new JoystickButton(XBShooter, Button.kA.value)
      .onFalse( new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorStow,ArmConstants.JeremyRennerStow)));
    
    new JoystickButton(XBShooter, Button.kX.value)
      .whileTrue( new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorX,ArmConstants.JeremyRennerX)));
    new JoystickButton(XBShooter, Button.kX.value)
      .onFalse( new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorStow,ArmConstants.JeremyRennerStow)));
    
    new JoystickButton(XBShooter, Button.kY.value)
      .whileTrue( new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorY,ArmConstants.JeremyRennerY)));
    new JoystickButton(XBShooter, Button.kY.value)
      .onFalse( new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorStow,ArmConstants.JeremyRennerStow)));
    
    new JoystickButton(XBShooter, Button.kB.value)
      .whileTrue( new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorB,ArmConstants.JeremyRennerB)));
    //Shelf angle changed from -177000 to -178000 after replay of match 47
    //Changed high angle from -191000 to -191500 
    //179 -> 174 practice field b4 64xs
    //174 -> 175 practice field b4 64
    new JoystickButton(XBShooter, Button.kB.value)
      .onFalse( new RunCommand(() -> m_arm.Angxtend(ArmConstants.extensionMotorStow,ArmConstants.JeremyRennerStow)));
    // new JoystickButton(XBShooter, Button.kY.value).whileTrue( new RunCommand(() -> m_arm.Angxtend(75,01)));
    
    new JoystickButton(XBShooter, Button.kLeftBumper.value).whileTrue( new RunCommand(() -> m_intake.jeremyRennerHug()));
    new JoystickButton(XBShooter, Button.kRightBumper.value).whileTrue( new RunCommand(() -> m_intake.jeremyRennerRelease()));
    new JoystickButton(XBShooter, Button.kLeftStick.value).whileTrue(new RunCommand(() -> m_arm.AlterLift()));
}

  /** n
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(3.1415)));
   
    // Run path following command, then stop at the end.
    // return new RunCommand(() -> m_arm.Angxtend(0,-190000)).until(m_arm::getLift) 
    //     .andThen(new RunCommand(() -> m_arm.Angxtend(-111,-190000)).until(m_arm::getReach))
    //     .andThen(new RunCommand(() -> m_intake.jeremyRennerHug()).withTimeout(0.5)
    //     .andThen(new RunCommand(() -> m_arm.Angxtend(-10,-190000)).until(m_arm::reachReturned))
    //     .andThen(new RunCommand(() -> m_arm.Angxtend(-1,-1000))
    //       .alongWith(midSwerveControllerCommand
    //     .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false)))));
    return m_chooser.getSelected();
    //removed chooser for armless m37
    //return redmidSwerveControllerCommand
    //  .andThen(new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false, false)));
  }

  private void generateSwerveCommands() {
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);    
    
    bluemidSwerveControllerCommand = new SwerveControllerCommand(
        Trajectories.bluemidTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    redmidSwerveControllerCommand = new SwerveControllerCommand(
        Trajectories.redmidTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    shortSwerveControllerCommand = new SwerveControllerCommand(
        Trajectories.shortTrajectory, //changed from short to redmid after m37
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);  
        
    noArmSwerveControllerCommand = new SwerveControllerCommand(
        Trajectories.shortTrajectory, //changed from short to redmid after m37
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
