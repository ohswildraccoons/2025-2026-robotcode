// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.MotorConstants;

import edu.wpi.first.math.geometry.Pose3d;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import static edu.wpi.first.units.Units.Amps;;




/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
/*  private final SparkFlex mFlexInake; 
  private final SparkFlex mFlexShooterPath;
  private final SparkFlex mFlexShooterOutake; */
  TalonFX m_IntakeFX;
   TalonFXConfiguration mTalonFXConfig = new TalonFXConfiguration();
   final TalonFXConfiguration commonConfigs = new TalonFXConfiguration()
   .withMotorOutput(
      new MotorOutputConfigs()
         .withNeutralMode(NeutralModeValue.Coast)
    
   )
   .withCurrentLimits(
      new CurrentLimitsConfigs()
         .withStatorCurrentLimit(Amps.of(120))
         .withStatorCurrentLimitEnable(true)
         .withSupplyCurrentLimit(Amps.of(40))
         .withSupplyCurrentLimitEnable(true)
  
   );
  //  .OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;


  // SparkMax m_intakeTraveler;
  // SparkMax m_intakeExtend;
  // SparkMax m_intakeTube;
  // TalonFX m_rightShooterBottom;
  // SparkMax m_rightShooterTop;

  // TalonFX m_leftShooterBottom;
  // TalonFX m_leftShooterTop;
   
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();


    
    // m_IntakeFX = new TalonF X(MotorConstants.kIntakeMotorPort);
    // m_rightShooterBottom = new TalonFX(MotorConstants.kLeftShooterMotorPort);
    // m_rightShooterTop = new SparkMax(MotorConstants.kLeftShooterMotorPortTop, MotorType.kBrushless);
    // m_leftShooterBottom = new TalonFX(MotorConstants.kLeftShooterMotorPort);
    // m_leftShooterTop = new TalonFX(MotorConstants.kLeftShooterMotorPortTop);
    // m_intakeTraveler = new SparkMax(MotorConstants.kIntakeTravellerMotorPort, MotorType.kBrushless);
    // m_intakeExtend = new SparkMax(MotorConstants.kIntakeExtendMotorPort, MotorType.kBrushless);
    // m_intakeTube = new  SparkMax(MotorConstants.kIntakeTubeMotorPort, MotorType.kBrushless);

    // m_IntakeFX.set(-0.6);
    // m_rightShooterBottom.set(1);
    // m_rightShooterTop.set(1);
    // m_leftShooterBottom.set(1);
    // m_leftShooterTop.set(-1);
    // m_intakeTraveler.set(0.85);
    // m_intakeExtend.set(0.0);
    // m_intakeTube.set(0.7);



  //   // m_TalonFX.setControl(new DutyCycleOut(1.0));
  //   var talonFXConfigurator = m_TalonFX.getConfigurator();
  //  talonFXConfigurator.apply(commonConfigs);

//    m_flexIntakeExtend = new SparkFlex(MotorConstants.kIntakeExtendMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless); 
// m_flexIntakeExtend.set(0.1);
    // mFlexInake  = new SparkFlex(MotorConstants.kIntakeMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    // mFlexInake.set(0);
   
    SmartDashboard.putData(CommandScheduler.getInstance());
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  Alliance currentAlliance = m_robotContainer.alliance();
  m_robotContainer.setIsRed(currentAlliance);

  SmartDashboard.putString("ALLIANCE", currentAlliance.toString());

  SmartDashboard.putString("IS_RED", "" + (m_robotContainer.isRed == -1));
 }

  /** This function is called once each tdime the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.getLeftShooterSubsytem().setDefaultCommand(m_robotContainer.getLeftShooterSubsytem().setSpeed(RotationsPerSecond.of(-55)));
    m_robotContainer.getRightShooterSubsytem().setDefaultCommand(m_robotContainer.getRightShooterSubsytem().setSpeed(RotationsPerSecond.of(-55)));  
   // m_robotContainer.getRightShooterSubsytem().setDefaultCommand(m_robotContainer.getRightShooterSubsytem().autoSetSpeed(m_robotContainer.getRightTurret().getGhostSupplier(), m_robotContainer.getRightTurret().getTurretFieldPosSupplier(() -> new Pose3d(m_robotContainer.getSwerveSubsystem().getPose()))));
    m_robotContainer.getSerializerSubsystem().setDefaultCommand(m_robotContainer.getSerializerSubsystem().runQ());

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
   // m_robotContainer.getLeftShooterSubsytem().setDefaultCommand( m_robotContainer.getLeftShooterSubsytem().setSpeed(RotationsPerSecond.of(0)));
   // m_robotContainer.getRightShooterSubsytem().setDefaultCommand( m_robotContainer.getRightShooterSubsytem().setSpeed(RotationsPerSecond.of(0)));
  }

}
