// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkFlexConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.GearRatios;
import frc.robot.Constants.MotorConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;


public class IntakeSubsystem extends SubsystemBase {

TalonFX rollerMotor;
TalonFXSimState rollerMotorSim;
TalonFXConfigurator rollerMotorConfigurator;
Pivot arm;
SmartMotorController sparkSmartMotorController;
boolean deployed = false;
BooleanSupplier deployedSupplier = ()->deployed;
CurrentLimitsConfigs rollerLimits = new CurrentLimitsConfigs();

  public IntakeSubsystem() {

    rollerMotor = new TalonFX(Constants.MotorConstants.kIntakeMotorPort);
    rollerMotorSim = new TalonFXSimState(rollerMotor);
    rollerMotorConfigurator = rollerMotor.getConfigurator();
    rollerLimits.SupplyCurrentLimit = 40; 
    rollerLimits.SupplyCurrentLimitEnable = true; 
    rollerMotorConfigurator.apply(rollerLimits);
  
    // rollEndcoder = rollerMotor.getEncoder();

   SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig()
  //.withControlMode(ControlMode.CLOSED_LOOP)
  .withSubsystem(this)
  .withClosedLoopController(2.0,0,0)//0.65, 0.00, 0.0)//
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(5.0,4.0,1.0),Sprocket.fromStages("24:48")))
  .withIdleMode(MotorMode.BRAKE)
  .withMotorInverted(false)
  // Setup Telemetry
  .withTelemetry("arm", TelemetryVerbosity.HIGH)
  // Power Optimization
  .withStatorCurrentLimit(Amps.of(40))
  .withSupplyCurrentLimit(Amps.of(30.0))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));

  
  // Vendor motor controller object
  SparkFlex RetractMC = new SparkFlex(Constants.MotorConstants.kIntakeExtendMotorPort, MotorType.kBrushless);

  // smcConfig.withExternalEncoder(RetractMC.getAbsoluteEncoder());

  // Create our SmartMotorController from our Spark and config with the NEO.
  sparkSmartMotorController = new SparkWrapper(RetractMC, DCMotor.getNeoVortex(1), smcConfig);

  PivotConfig                m_config         = new PivotConfig(sparkSmartMotorController)
   //   .withSoftLimits(Degrees.of(0.0), Degrees.of(90.0)) // Soft limits for the arm, these will be enforced in code but not by the motor controller
      .withStartingPosition(Degrees.of(0.0)) // Starting position of the Pivot
      .withHardLimit(Degrees.of(-10.0), Degrees.of(140.0)) // Hard limit bc wiring prevents infinite spinning
      .withTelemetry("intake Pivot", TelemetryVerbosity.HIGH) // Telemetry
      .withMOI(Feet.of(0.25), Pounds.of(15));// MOI Calculation

  // Arm Mechanism
   arm = new Pivot(m_config);

    
  }
 
  /**
   * Set the angle of the arm.
   * @param angle Angle to go to.
   */
  public Command setAngle(Supplier<Angle> angle) {
    return arm.setAngle(() -> {
      return angle.get();
    });
  }
  
  public Command protectedDeploy(Angle target, Angle cutoff) {
    if (arm.getAngle().isNear(target, cutoff))
    {
      return arm.set(0.0);
    }
    else 
    { 
      return arm.setAngle(target);
    }
  }

  /**
   * Set the angle of the arm, ends the command but does not stop the arm when the arm reaches the setpoint.
   * @ param angle Angle to go to.
   * @ param tolerance Angle tolerance for completion.
   *  @ return A Command
   */
  //         public Command setAngleAndStop(Angle angle, Angle tolerance) { return arm.runTo(angle, tolerance);}
  
  /**
   * Set arm closed loop controller to go to the specified mechanism position.
   * @ param angle Angle to go to.
   */
  //         public void setAngleSetpoint(Angle angle) { arm.setMechanismPositionSetpoint(angle); }

  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) { return arm.set(dutycycle);}

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() { return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command check(){
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {
            if (deployed) {
                rollerMotor.set(0.0);
            } else {
                rollerMotor.set(0.0);
            }}
          );
  }

public Command toggleDeploy() {
  return runOnce(() -> {
    deployed = !deployed;

    if (deployedSupplier.getAsBoolean()) {
      SmartDashboard.putString("deployed status", "deployed");
    } else {
      SmartDashboard.putString("deployed status", "undeployed");
    }

    check();
  });
}

  public Command stopMotors() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {

          rollerMotor.set(0);

          /* one-time action goes here */
        });
  }
 public Command rollRollers() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {

          rollerMotor.set(-0.5);

          /* one-time action goes here */
        });
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   *  return value of some boolean subsystem state, such as a digital sensor.
   */
  //public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    //return false;
  //}

  // public Command unjam() {
  //   return runOnce(() -> {
  //      set(-1.0)
  //     .andThen(new WaitCommand(.25))
  //     .andThen(rollMotors())
  //         });

  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("Arm Angle: " + (arm.getAngle().in(Degrees)));
    // System.out.println("Arm setpoint: " + arm.getMotor().getMechanismPositionSetpoint());

    arm.updateTelemetry();
    SmartDashboard.putString("arm Angle", arm.getAngle().toShortString());
    SmartDashboard.putString("arm Angle --2", arm.getMotor().getRotorPosition().toShortString());
    SmartDashboard.putNumber("roller motors RPM", rollerMotor.getVelocity().getValueAsDouble() * 60);
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    arm.simIterate();
  }

public void setCurrentLimit(double limit) {

    arm.getMotorController().setSupplyCurrentLimit(Amps.of(limit));
    rollerLimits.SupplyCurrentLimit = limit; 
    rollerMotorConfigurator.apply(rollerLimits);
  
  
}
}