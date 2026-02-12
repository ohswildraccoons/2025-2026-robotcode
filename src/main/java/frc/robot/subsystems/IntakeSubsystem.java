// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.MotorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
  SparkFlex IntakeExtendMotor;
  SparkFlex IntakeRollerMotor;
   SparkFlexConfig IntakeConfig;
   boolean deployed;
  SparkClosedLoopController ExtendController = IntakeExtendMotor.getClosedLoopController();
  

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    IntakeExtendMotor = new SparkFlex(MotorConstants.kIntakeExtendMotorPort, MotorType.kBrushless);

    IntakeRollerMotor = new SparkFlex(MotorConstants.kIntakeMotorPort, MotorType.kBrushless);

    SparkFlexConfig IntakeConfig = new SparkFlexConfig();
    SparkFlexConfig ExtendConfig = new SparkFlexConfig();

    ExtendConfig.closedLoop
        .p(0) //we need to figure out the gear ratio then # of units per rotation
        .i(0.0)
        .d(0.0)
        .outputRange(kMinOutput, kMaxOutput); //limits(?)
    

    IntakeConfig.smartCurrentLimit(40);
    IntakeConfig.openLoopRampRate(0.125);

    IntakeConfig.idleMode(IdleMode.kBrake);


    IntakeConfig.inverted(false);
    IntakeRollerMotor.set(0);
    IntakeExtendMotor.set(0);//sets speed for Extension motor to 100%


  }

  /*
   * Example command factory method.
   *
   * return a command
   */

   
  public Command deployRollers() {

        return runOnce(()->{

  ExtendController.setSetpoint(0, ControlType.kPosition);

    });
  }

public Command speed(double x){

      return run(()->{
         
      IntakeRollerMotor.set(x);

      });

};
      
/* public Command runRollers() {

        return run(()->{
         IntakeRollerMotor.set(1);     // runs when command stops being called
            
         } );
}*/
  

        
 @Override
  public void periodic() {
    // SmartDashboard.putNumber("Motorspeed", IntakeExtendMotor.get);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
