// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TurretConstants;


import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsytem;



// import dev.doglog.DogLog;
// import dev.doglog.DogLogOptions;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * 
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // // The robot's subsystems and commands are defined here...
  // private final ShooterSubsytem m_shooterSubsystem = new ShooterSubsytem();
  private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem();
  private final TurretSubsystem m_TurretSubsystem = new TurretSubsystem(TurretConstants.leftTurretMetersX, TurretConstants.leftTurretMetersY, TurretSubsystem.TurretSide.LEFT);
  // private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  Pose3d robotPose = new Pose3d();

  private final SendableChooser<Command> autoChooser;
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
 private final CommandXboxController m_mechController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort + 1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();

    m_swerveDrive.setDefaultCommand(
      m_swerveDrive.driveCommand(
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getLeftY() * -1,
        () -> m_driverController.getRightX()
        )
    );
    // m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.set(0)); 
    m_TurretSubsystem.setDefaultCommand(
        m_TurretSubsystem.targettingCommand(
          () -> new Pose3d(m_swerveDrive.getPose()),
          m_swerveDrive
      )
    );


    //  m_IntakeSubsystem.setDefaultCommand(m_IntakeSubsystem.runRollers(m_mechController.x().getAsBoolean()));
     
    

    


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
  boolean intakeToggle = false;
  private void configureBindings() {

    
      //       Trigger leftTrigger = new Trigger(() -> m_mechController.getLeftTriggerAxis() > 0.2);
      //  leftTrigger.onTrue(new InstantCommand(() -> {
      //   intakeToggle = !intakeToggle;
      //   if (intakeToggle) {
      //       m_IntakeSubsystem.runRollers();
      //   } else {
      //       m_IntakeSubsystem.stopRollers();
      //   }
      //  }));
      // m_mechController.rightBumper().onTrue(m_IntakeSubsystem.DeployUndeplyRollers());
    
    m_mechController.x().whileTrue(m_TurretSubsystem.setManualTarget(FieldConstants.blueLeftDeposit));
    m_mechController.y().whileTrue(m_TurretSubsystem.setManualTarget(FieldConstants.blueRightDeposit));
    m_mechController.a().whileTrue(m_TurretSubsystem.setManualTarget(FieldConstants.blueHub));
    m_mechController.b().whileTrue(m_TurretSubsystem.setAutoTargettingOn());



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
}
