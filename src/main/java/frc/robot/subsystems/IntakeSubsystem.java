// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.thethriftybot.util.Conversion;

import frc.robot.Constants;
import frc.robot.Constants.Conversions;
import frc.robot.Constants.GearRatios;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TargetPositions;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

SparkFlex IntakeExtendMotor;
SparkFlex IntakeRollerMotor;
    SparkFlexConfig RollerConfig;
    SparkFlexConfig ExtendConfig;

private SparkClosedLoopController ExtendController;


//sim motors
private SparkFlexSim IntakeRollerSimMotor;
private SparkFlexSim IntakeExtendSimMotor;
//encoder
private RelativeEncoder encoder;
//doubles
double simPosition = TargetPositions.ROLLER_DEPLOYED_POSITION;
double lastSimPosition = simPosition;
boolean deployed = false;



  public IntakeSubsystem() {

IntakeExtendMotor = new SparkFlex(MotorConstants.kIntakeExtendMotorPort, MotorType.kBrushless);
encoder = IntakeExtendMotor.getEncoder(); 
IntakeRollerMotor = new SparkFlex(MotorConstants.kIntakeMotorPort, MotorType.kBrushless);

  RollerConfig = new SparkFlexConfig();
   ExtendConfig = new SparkFlexConfig();



ExtendController = IntakeExtendMotor.getClosedLoopController();
    ExtendConfig.closedLoop
        .p(0.003) 
        .i(0)
        .d(0.001)
        .outputRange(-1, 1); //limits
        
    RollerConfig.smartCurrentLimit(40);
    ExtendConfig.smartCurrentLimit(40);
    RollerConfig.openLoopRampRate(0.125);
    ExtendConfig.openLoopRampRate(0.125);

    RollerConfig.idleMode(IdleMode.kBrake);
    ExtendConfig.idleMode(IdleMode.kBrake);


    RollerConfig.inverted(false);
    ExtendConfig.inverted(false);


    IntakeExtendMotor.configure(ExtendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    IntakeRollerMotor.configure(RollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

 
    IntakeRollerSimMotor = new SparkFlexSim(IntakeRollerMotor, DCMotor.getKrakenX44(1));//correct motor?
    IntakeExtendSimMotor = new SparkFlexSim(IntakeExtendMotor, DCMotor.getNeoVortex(1));

  }


  public Command DeployUndeplyRollers(){

    return runOnce(()->{
           deployed = !deployed;
 if (deployed == true){
      ExtendController.setSetpoint(
       (TargetPositions.ROLLER_RETRACT_POSITION * Conversions.DEGREES_TO_ROT * GearRatios.INTAKE_DEPLOY_GEAR_RATIO * GearRatios.PULLEY_RATIO), 
        ControlType.kPosition);  
  }
 else{
      ExtendController.setSetpoint((TargetPositions.ROLLER_DEPLOYED_POSITION * Conversions.DEGREES_TO_ROT * GearRatios.INTAKE_DEPLOY_GEAR_RATIO * GearRatios.PULLEY_RATIO),
     ControlType.kPosition);
    }
 });
}

public Command runRollers() {

  return run(()->{ 
   IntakeRollerMotor.set(1);  
 } );
};
public Command stopRollers(){
return runOnce(()->{
IntakeRollerMotor.set(0);
});
};

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * return value of some boolean subsystem state, such as a digital sensor.
   */
  /*public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

     SmartDashboard.putBoolean("Are we Deployed?", deployed);
  SmartDashboard.putNumber("Intake encoder Position", encoder.getPosition());
  SmartDashboard.putNumber("Roller Velocity", (IntakeRollerMotor.getAppliedOutput() * Conversions.SPEED_FACTOR));
      double intakePosition = RobotBase.isSimulation() ? simPosition : encoder.getPosition() * Conversions.ROT_TO_DEGREES;
  SmartDashboard.putNumber("Intake Position", intakePosition);
  SmartDashboard.putNumber("Intake applied output", IntakeExtendMotor.getAppliedOutput()*40);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    

    double targetPosition = deployed ? TargetPositions.ROLLER_RETRACT_POSITION : 
    TargetPositions.ROLLER_DEPLOYED_POSITION;

  ExtendController.setSetpoint(targetPosition*Conversions.DEGREES_TO_ROT*GearRatios.INTAKE_DEPLOY_GEAR_RATIO*GearRatios.PULLEY_RATIO, ControlType.kPosition);

  double ExtendOutput = IntakeExtendMotor.getAppliedOutput();
simPosition += ExtendOutput * Conversions.SPEED_FACTOR * .02; // degrees/second (tune); this goes into constants
    if(simPosition > TargetPositions.ROLLER_DEPLOYED_POSITION) simPosition = TargetPositions.ROLLER_DEPLOYED_POSITION; //make these constants
    if(simPosition < TargetPositions.ROLLER_RETRACT_POSITION) simPosition = TargetPositions.ROLLER_RETRACT_POSITION; //deployed=90

     double motorVelocity = ((simPosition - lastSimPosition)/0.02);
 lastSimPosition = simPosition; 

  IntakeExtendSimMotor.iterate( motorVelocity , 12, 0.02); 
   
    double rollerOutput = IntakeRollerMotor.getAppliedOutput();
double rollerVelocity = rollerOutput * Conversions.SPEED_FACTOR; //conversion factor; arbitrary rn

 IntakeRollerSimMotor.iterate( rollerVelocity ,  12, 0.02);

      SmartDashboard.putNumber("Sim Position", simPosition);
      SmartDashboard.putNumber("Motor Velocity", motorVelocity);
      SmartDashboard.putNumber("Motor Velocity", rollerVelocity);
    //TODO Alert Posting 
    // Failure modes we can deal with:
    // Rollers are not rolling
    //deploy or undeploy is stuck
    //deploy or undeploy has exceeded limits
    // j
    
  }
}
