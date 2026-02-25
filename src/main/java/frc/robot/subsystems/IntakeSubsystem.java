// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.Conversions;
import frc.robot.Constants.GearRatios;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TargetPositions;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
  SparkFlex IntakeExtendMotor;
  SparkFlex IntakeRollerMotor;
   SparkFlexConfig IntakeConfig;
 private final FlywheelSim m_flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), 2.0, 1.0), new DCMotor(12, 3.6, 211, 3.6, 710.4, 1));
private SparkFlexSim IntakeRollerSimMotor;
private SparkFlexSim IntakeExtendSimMotor;
private SparkFlexSim IntakeRetractSimMotor;
private SparkClosedLoopController ExtendController;
private AbsoluteEncoder encoder;
 
  

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    IntakeExtendMotor = new SparkFlex(MotorConstants.kIntakeExtendMotorPort, MotorType.kBrushless);

    IntakeRollerMotor = new SparkFlex(MotorConstants.kIntakeMotorPort, MotorType.kBrushless);

    SparkFlexConfig IntakeConfig = new SparkFlexConfig();
    SparkFlexConfig ExtendConfig = new SparkFlexConfig();
  
    ExtendController = IntakeExtendMotor.getClosedLoopController();
    ExtendConfig.closedLoop
        .p(0.003) //we need to figure out the gear ratio then # of units per rotation
        .i(0)
        .d(0.001)
        .outputRange(0, 1); //limits
        
    

    IntakeConfig.smartCurrentLimit(40);
    IntakeConfig.openLoopRampRate(0.125);

    IntakeConfig.idleMode(IdleMode.kBrake);


    IntakeConfig.inverted(false);
    IntakeRollerMotor.set(0);
    IntakeExtendMotor.set(0);//sets speed for Extension motor to 100%

    IntakeRollerSimMotor = new SparkFlexSim(IntakeRollerMotor, DCMotor.getNeoVortex(3));
    IntakeExtendSimMotor = new SparkFlexSim(IntakeExtendMotor, DCMotor.getNeoVortex(3));
    IntakeRetractSimMotor = new SparkFlexSim(IntakeExtendMotor, DCMotor.getNeoVortex(3));


    encoder = IntakeExtendMotor.getAbsoluteEncoder();


  }

  /*
   * Example command factory method.
   *
   * return a command
   */

   
  public Command deployRollers() {

        return runOnce(() ->{

  ExtendController.setSetpoint((TargetPositions.ROLLER_DEPLOYED_POSITION * Conversions.DEGREES_TO_ROT * GearRatios.INTAKE_DEPLOY_GEAR_RATIO * GearRatios.PULLEY_RATIO), ControlType.kPosition);

          
    encoder.getPosition();

   if(encoder.getPosition() < (TargetPositions.ROLLER_LIM_POSITION * Conversions.DEGREES_TO_ROT * GearRatios.INTAKE_DEPLOY_GEAR_RATIO * GearRatios.PULLEY_RATIO)){
 ExtendController.setSetpoint((TargetPositions.ROLLER_DEPLOYED_POSITION * Conversions.DEGREES_TO_ROT * GearRatios.INTAKE_DEPLOY_GEAR_RATIO * GearRatios.PULLEY_RATIO), ControlType.kPosition);

  }
        else{
    
   }

    });
  }

  public Command retractRollers(){
      return runOnce(()->{

    ExtendController.setSetpoint(TargetPositions.ROLLER_RETRACT_POSITION * Conversions.DEGREES_TO_ROT * GearRatios.INTAKE_DEPLOY_GEAR_RATIO * GearRatios.PULLEY_RATIO, ControlType.kPosition);//for standarization

      });

  }

public Command speed(double x){

      return run(()->{
         
      IntakeRollerMotor.set(x);

      
      });

};
      
/ public Command runRollers() {

        return run(()->{
         IntakeRollerMotor.set(1);     // runs when command stops being called 
            
         } );
}*/
  

        
 @Override
  public void periodic() {
     SmartDashboard.putNumber("Motorspeed", IntakeExtendMotor.get);
  }

   @Override
   public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
m_flywheelSim.setInput(IntakeRollerSimMotor.getVelocity() * RobotController.getBatteryVoltage());
m_flywheelSim.update(.02);
IntakeRollerSimMotor.iterate( 1,  12, 0.02);

IntakeExtendSimMotor.iterate( 1, 12, 0.02);

IntakeRetractSimMotor.iterate( 1, 12, 0.02);

  }
} 
//why so serious?
//joker stay yound beautiful and unique
