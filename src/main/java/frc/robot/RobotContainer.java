// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TurretConstants;

import edu.wpi.first.wpilibj.DriverStation.Alliance;


import frc.robot.commands.Autos;
import frc.robot.subsystems.CurrentManagementSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.reflect.Field;
import java.util.Optional;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.serializerSubsystem;
import frc.robot.subsystems.serializerSubsystem;
import frc.robot.subsystems.ShooterSubsytem;



 import dev.doglog.DogLog;
 import dev.doglog.DogLogOptions;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * 
 * the {@link Robot}a
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // // The robot's subsystems and commands are defined here...
  public static Optional<Alliance> ALLIANCE = Optional.empty();

  private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem();

  private final TurretSubsystem m_TurretSubsystemLeft = 
    new TurretSubsystem(TurretConstants.leftTurretMetersX, TurretConstants.leftTurretMetersY, 
                        TurretSubsystem.TurretSide.LEFT, Constants.MotorConstants.kLeftTurretMotorPort);
    private final TurretSubsystem m_TurretSubsystemRight = 
    new TurretSubsystem(TurretConstants.rightTurretMetersX, TurretConstants.rightTurretMetersX, 
                        TurretSubsystem.TurretSide.RIGHT, Constants.MotorConstants.kRightTurretMotorPort);


  private final ShooterSubsytem m_ShooterSubsystemLeft = 
    new ShooterSubsytem(Constants.MotorConstants.kLeftShooterMotorPortLeft, Constants.MotorConstants.kLeftShooterMotorPortRight);
  private final ShooterSubsytem m_ShooterSubsystemRight = 
    new ShooterSubsytem(Constants.MotorConstants.kRightShooterMotorPortLeft, Constants.MotorConstants.kRightShooterMotorPortRight);
  
  // private final CurrentManagementSubsystem m_CurrentManagementSubsystem = new CurrentManagementSubsystem(m_swerveDrive, getIntake(), getSerializerSubsystem(), m_TurretSubsystemRight, m_TurretSubsystemLeft, m_ShooterSubsystemRight, m_ShooterSubsystemLeft); //needs to be last one

  //constant issue// private final ShooterSubsytem m_shooterSubsystem = new ShooterSubsytem(Constants.MotorConstants.kShooterMotorPort, Constants.MotorConstants.kShooterMotorPortTop);
  private final serializerSubsystem m_serializerSubsystem = new serializerSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  Pose3d robotPose = new Pose3d();

  private final SendableChooser<Command> autoChooser;
  int isRed;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
 private final CommandXboxController m_mechController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort + 1); //TODO: are you sure you want to do it this way? port and port+1 or just hardcode it



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    NamedCommands.registerCommand("deploy rollers", m_IntakeSubsystem.setAngle(() -> Degrees.of(0)));
    NamedCommands.registerCommand("undeploy rollers", m_IntakeSubsystem.setAngle(() -> Degrees.of(90)));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();

    m_swerveDrive.setDefaultCommand(
      m_swerveDrive.driveCommand(
      () -> m_driverController.getLeftY() * isRed, //forward back  
      () -> m_driverController.getLeftX() * isRed, // left right
        
        
        
        () -> m_driverController.getRightX()//rotation
        )
    );

    //  m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.autoSetVelocityOfFire(m_TurretSubsystem.getTarget(), () -> new Pose3d(m_swerveDrive.getPose()))); 

    m_TurretSubsystemLeft.setDefaultCommand(
        m_TurretSubsystemLeft.targettingCommand(
          () -> new Pose3d(m_swerveDrive.getPose()),
          m_swerveDrive,
          alliance()
      )
    );
    // m_ShooterSubsystemLeft.setDefaultCommand(m_ShooterSubsystemLeft.setSpeed(RotationsPerSecond.of(-7500)));
    // m_ShooterSubsystemLeft.setDefaultCommand(m_ShooterSubsystemLeft.autoSetVelocityOfFire(m_TurretSubsystemLeft.getGhostSupplier(), m_TurretSubsystemLeft.getTurretFieldPosSupplier( () -> new Pose3d(m_swerveDrive.getPose()))));
    
    m_TurretSubsystemRight.setDefaultCommand(
        m_TurretSubsystemRight.targettingCommand(
          () -> new Pose3d(m_swerveDrive.getPose()),
          m_swerveDrive,
          alliance()
      )
    );
        // m_ShooterSubsystemRight.setDefaultCommand(m_ShooterSubsystemRight.setSpeed(RotationsPerSecond.of(-7500)));
    // m_ShooterSubsystemRight.setDefaultCommand(m_ShooterSubsystemRight.autoSetVelocityOfFire(m_TurretSubsystemRight.getGhostSupplier(), m_TurretSubsystemRight.getTurretFieldPosSupplier( () -> new Pose3d(m_swerveDrive.getPose()))));
    
    m_serializerSubsystem.setDefaultCommand(m_serializerSubsystem.runQ());
  }

  public void setIsRed(Alliance alliance) {
    isRed = (alliance == Alliance.Red) ? -1 : 1;
  }

  public static Alliance alliance() {
      if (ALLIANCE.isPresent()) {
          SmartDashboard.putString("Alliance", ALLIANCE.get().toString());
          return ALLIANCE.get();
      }

      if (DriverStation.getAlliance().isPresent()) {
          ALLIANCE = Optional.of(DriverStation.getAlliance().get());
          return ALLIANCE.get();
      }

      return Alliance.Blue; // default
  }


  /**>
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_driverController.rightBumper().onTrue(
      new SequentialCommandGroup(
        // m_IntakeSubsystem.stopMotors(),
        m_IntakeSubsystem.setAngle(() -> Degrees.of(-10)) ////TODO:game day hack (angles)
      )
    );
    m_driverController.leftBumper().onTrue(    
      new SequentialCommandGroup(
        // m_IntakeSubsystem.rollRollers(),
        m_IntakeSubsystem.setAngle(() -> Degrees.of(130))
      )
    );
    
    if (alliance() == Alliance.Blue){
      m_mechController.x().whileTrue( new ParallelCommandGroup(
        m_TurretSubsystemLeft.setManualTarget(FieldConstants.blueLeftDeposit),
        m_TurretSubsystemRight.setManualTarget(FieldConstants.blueLeftDeposit)
        ));
      m_mechController.b().whileTrue( new ParallelCommandGroup(
        m_TurretSubsystemLeft.setManualTarget(FieldConstants.blueRightDeposit),
        m_TurretSubsystemRight.setManualTarget(FieldConstants.blueRightDeposit)
        ));
      m_mechController.y().whileTrue(new ParallelCommandGroup(
        m_TurretSubsystemLeft.setManualTarget(FieldConstants.blueHub),
        m_TurretSubsystemRight.setManualTarget(FieldConstants.blueHub)
        ));
    }else if (alliance() == Alliance.Red){
      m_mechController.x().whileTrue( new ParallelCommandGroup(
        m_TurretSubsystemLeft.setManualTarget(FieldConstants.redLeftDeposit),
        m_TurretSubsystemRight.setManualTarget(FieldConstants.redLeftDeposit)
        ));
      m_mechController.b().whileTrue( new ParallelCommandGroup(
        m_TurretSubsystemLeft.setManualTarget(FieldConstants.redRightDeposit),
        m_TurretSubsystemRight.setManualTarget(FieldConstants.redRightDeposit)
        ));
      m_mechController.y().whileTrue(new ParallelCommandGroup(
        m_TurretSubsystemLeft.setManualTarget(FieldConstants.redHub),
        m_TurretSubsystemRight.setManualTarget(FieldConstants.redHub)
        ));
    }
    m_mechController.a().whileTrue(new ParallelCommandGroup(
      m_TurretSubsystemLeft.setSplitTarget(alliance()),
      m_TurretSubsystemRight.setSplitTarget(alliance())
    ));
    m_mechController.rightBumper().whileTrue(new ParallelCommandGroup(
      m_TurretSubsystemLeft.setAutoTargettingOn(),
      m_TurretSubsystemRight.setAutoTargettingOn()
    ));

    m_mechController.leftBumper().onChange(m_serializerSubsystem.activateJam());
    m_mechController.povDown().onChange(new ParallelCommandGroup(
      m_IntakeSubsystem.stopMotors(),
      m_serializerSubsystem.hardStopMotor()
      ));

    
    

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }


  public TurretSubsystem getLeftTurret()
  {
    return m_TurretSubsystemLeft;
  }

   public TurretSubsystem getRightTurret()
  {
    return m_TurretSubsystemLeft;
  }

  public ShooterSubsytem getLeftShooterSubsytem()
  {
    return m_ShooterSubsystemLeft;
  }

  public ShooterSubsytem getRightShooterSubsytem()
  {
    return m_ShooterSubsystemRight;
  }
 
  public IntakeSubsystem getIntake()
  {
    return m_IntakeSubsystem;
  }
 
  public serializerSubsystem getSerializerSubsystem()
  {
    return m_serializerSubsystem;
  }
 
  public SwerveSubsystem getSwerveSubsystem()
  {
    return m_swerveDrive;
  }
}
