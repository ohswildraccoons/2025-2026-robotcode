// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
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


public class IntakeSubsystem extends SubsystemBase {
  //two total motors
  SparkFlex IntakeExtendMotor;
  SparkFlex IntakeRollerMotor;
   SparkFlexConfig RollerConfig;
   SparkFlexConfig ExtendConfig;

   //the sim needs workin' on
 private final FlywheelSim m_flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), 2.0, 1.0), new DCMotor(12, 3.6, 211, 3.6, 710.4, 1));
private SparkFlexSim IntakeRollerSimMotor;
private SparkFlexSim IntakeExtendSimMotor;
private SparkFlexSim IntakeRetractSimMotor;
private SparkClosedLoopController ExtendController;
//can it be two encoders??

boolean deployed=false;
private SparkAbsoluteEncoder encoder1;
private SparkAbsoluteEncoder encoder2;

private double simPosition = TargetPositions.ROLLER_DEPLOYED_POSITION;
double lastSimPosition = simPosition;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    IntakeExtendMotor = new SparkFlex(MotorConstants.kIntakeExtendMotorPort, MotorType.kBrushless);
encoder1 = IntakeExtendMotor.getAbsoluteEncoder();
    IntakeRollerMotor = new SparkFlex(MotorConstants.kIntakeMotorPort, MotorType.kBrushless);
//encoder2 = IntakeRollerMotor.getAbsoluteEncoder();

    RollerConfig = new SparkFlexConfig();
    ExtendConfig = new SparkFlexConfig();
  
     //configure(ExtendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) <-- doesn't work with ExtendConfig bc /= SparkBase

    //IntakeExtendMotor.configure(ExtendConfig, null, null)
    //IntakeExtendMotor.configure(ExtendConfig, null, null)

    //speed based on 'error' / PID
    //IntakeExtendMotor.configure(ExtendConfig, );//what's reset and persist mode?
    ExtendController = IntakeExtendMotor.getClosedLoopController();
    ExtendConfig.closedLoop
        .p(0.003) 
        .i(0)
        .d(0.001)
        .outputRange(0, 1); //limits
        
    RollerConfig.smartCurrentLimit(40);
    ExtendConfig.smartCurrentLimit(40);
    RollerConfig.openLoopRampRate(0.125);
    ExtendConfig.openLoopRampRate(0.125);

    RollerConfig.idleMode(IdleMode.kBrake);
    ExtendConfig.idleMode(IdleMode.kBrake);


    RollerConfig.inverted(false);
    ExtendConfig.inverted(false);
 
    IntakeRollerSimMotor = new SparkFlexSim(IntakeRollerMotor, DCMotor.getNeoVortex(1));
    IntakeExtendSimMotor = new SparkFlexSim(IntakeExtendMotor, DCMotor.getNeoVortex(1));
     // IntakeExtendMotor.configure(ExtendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // k or kNo

    
  }

/*  public Command deployRollers() {

     return runOnce(() ->{

     
 }; */

  /*public Command retractRollers(){
      return runOnce(()->{

    ExtendController.setSetpoint
    (TargetPositions.ROLLER_RETRACT_POSITION * Conversions.DEGREES_TO_ROT * GearRatios.INTAKE_DEPLOY_GEAR_RATIO * GearRatios.PULLEY_RATIO,
     ControlType.kPosition);//for standarization
    });
 };
 */
 public Command DeployUndeplyRollers(){

    return runOnce(()->{
     
     deployed = !deployed; //! turns t->F, F->T
     // if ( encoder1.getPosition()>90 ){/*< TargetPositions.ROLLER_LIM_START*Conversions.DEGREES_TO_ROT*GearRatios.INTAKE_DEPLOY_GEAR_RATIO*GearRatios.PULLEY_RATIO*/
     //    deployed = false;
     //   }
     //  else if(encoder1.getPosition()<90){ /*> TargetPositions.ROLLER_LIM_START*Conversions.DEGREES_TO_ROT*GearRatios.INTAKE_DEPLOY_GEAR_RATIO*GearRatios.PULLEY_RATIO*/
     //     deployed = true;
     //   };
      // false/true 

    if (deployed == true){
      /*deployRollers();*/ 
      ExtendController.setSetpoint(
       (TargetPositions.ROLLER_RETRACT_POSITION * Conversions.DEGREES_TO_ROT * GearRatios.INTAKE_DEPLOY_GEAR_RATIO * GearRatios.PULLEY_RATIO), 
        ControlType.kPosition);}
    else{
     /* retractRollers(); */ExtendController.setSetpoint((TargetPositions.ROLLER_DEPLOYED_POSITION * Conversions.DEGREES_TO_ROT * GearRatios.INTAKE_DEPLOY_GEAR_RATIO * GearRatios.PULLEY_RATIO),
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
 @Override
  public void periodic() {
  SmartDashboard.putNumber("Sim Position (deg)", simPosition);
  SmartDashboard.putBoolean("Are we Deployed?", deployed);
  SmartDashboard.putNumber("Motor Velocity", encoder1.getVelocity());
  SmartDashboard.putNumber("Roller Velocity", (IntakeRollerMotor.getAppliedOutput() * Conversions.SPEED_FACTOR));
      double intakePosition = RobotBase.isSimulation() ? simPosition : encoder1.getPosition() * Conversions.ROT_TO_DEGREES;

  SmartDashboard.putNumber("Intake Position", intakePosition);
  };
   //degrees
   @Override
   public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation



double ExtendOutput = IntakeExtendMotor.getAppliedOutput();
 
simPosition += ExtendOutput * Conversions.SPEED_FACTOR * .02; // degrees/second (tune); this goes into constants
    if(simPosition > TargetPositions.ROLLER_DEPLOYED_POSITION) simPosition = TargetPositions.ROLLER_DEPLOYED_POSITION; //make these constants
    if(simPosition < TargetPositions.ROLLER_RETRACT_POSITION) simPosition = TargetPositions.ROLLER_RETRACT_POSITION;

   IntakeExtendSimMotor.setPosition(simPosition * Conversions.DEGREES_TO_ROT);

 double targetPosition = deployed ? TargetPositions.ROLLER_RETRACT_POSITION*Conversions.DEGREES_TO_ROT*GearRatios.INTAKE_DEPLOY_GEAR_RATIO*GearRatios.PULLEY_RATIO : TargetPositions.ROLLER_DEPLOYED_POSITION;
 ExtendController.setReference(targetPosition, ControlType.kPosition);

double rollerOutput = IntakeRollerMotor.getAppliedOutput();
double rollerVelocity = rollerOutput * 400; //comversion factor; arbitrary rn


m_flywheelSim.setInput(IntakeRollerSimMotor.getVelocity() * RobotController.getBatteryVoltage());
m_flywheelSim.update(.02);
IntakeRollerSimMotor.iterate( rollerVelocity ,  12, 0.02);


double motorVelocity = (simPosition - lastSimPosition)/0.02;
lastSimPosition = simPosition;
SmartDashboard.putNumber("Motor Velocity", motorVelocity);


IntakeExtendSimMotor.iterate( encoder1.getVelocity() , 12, 0.02);
//IntakeRetractSimMotor.iterate( 1, 12, 0.02);
  }
}


