// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsytem;

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
  // The robot's subsystems and commands are defined here...
  private final ShooterSubsytem m_shooterSubsystem = new ShooterSubsytem();
  private final SwerveSubsystem m_swerveDrive = new SwerveSubsystem();
  private final TurretSubsystem m_TurretSubsystem = new TurretSubsystem();
 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_swerveDrive.setDefaultCommand(
        m_swerveDrive.driveCommand(
            () -> m_driverController.getLeftX() * -1,
            () -> m_driverController.getLeftY() * -1,
            () -> m_driverController.getRightX() * -1));

    // Configure the trigger bindings
    configureBindings();

    
    // Set the default command to force the shooter rest.
    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.set(0)); 
    // Set the default command to force the arm to go to 0.
    m_TurretSubsystem.setDefaultCommand(m_TurretSubsystem.setAngle(Degrees.of(0)));

  }

  /**
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
            // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
            // new Trigger(m_exampleSubsystem::exampleCondition)
            //     .onTrue(new ExampleCommand(m_exampleSubsystem));

            // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // // Schedule `setVelocity` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.a().whileTrue(m_shooterSubsystem.setVelocity(RPM.of(60)));
    // m_driverController.b().whileTrue(m_shooterSubsystem.setVelocity(RPM.of(300)));
    // // Schedule `set` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.x().whileTrue(m_shooterSubsystem.set(0.3));
    // m_driverController.y().whileTrue(m_shooterSubsystem.set(-0.3));

    
    // Schedule `setAngle` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.a().whileTrue(m_TurretSubsystem.setAngle(m_TurretSubsystem.getAngle().minus(Degrees.of(5))));
    // m_driverController.b().whileTrue(m_TurretSubsystem.setAngle(m_TurretSubsystem.getAngle().plus(Degrees.of(5))));
    m_driverController.a().whileTrue(m_TurretSubsystem.setAngle(m_TurretSubsystem.getAngle().plus(Degrees.of(90))));
    m_driverController.b().whileTrue(m_TurretSubsystem.setAngle(m_TurretSubsystem.getAngle().minus(Degrees.of(90))));
    // Schedule `set` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.x().whileTrue(m_TurretSubsystem.set(0.3));
    m_driverController.y().whileTrue(m_TurretSubsystem.set(-0.3));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return Autos.exampleAuto(null);
  }
}
