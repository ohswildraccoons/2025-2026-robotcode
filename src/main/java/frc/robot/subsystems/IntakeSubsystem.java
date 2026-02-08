// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
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

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    IntakeExtendMotor = new SparkFlex(MotorConstants.kIntakeExtendMotorPort, MotorType.kBrushless);

    IntakeRollerMotor = new SparkFlex(MotorConstants.kIntakeMotorPort, MotorType.kBrushless);

    IntakeConfig = new SparkFlexConfig();

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

public Command RunRollers(){

  return new Command(){

    if(deployed==false){
      deployRunRollers()
    };
  };

};
   
  public Command deployRunRollers() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

   return new Command() {
     @Override
     public void initialize() {
       IntakeExtendMotor.set(1.0);
       IntakeRollerMotor.set(0);   // one-time action
        }

     @Override
     public void end(boolean interrupted) {
         IntakeExtendMotor.set(0);
         IntakeRollerMotor.set(1.0);     // runs when command stops being called
            deployed = true;
     }
   };
  }

   public Command unDeployRunRollers() {

     return new Command() {
     @Override
     public void initialize() {
       IntakeExtendMotor.set(-1.0);
       IntakeRollerMotor.set(0);   // one-time action
        }

     @Override
    public void end(boolean interrupted) {
         IntakeExtendMotor.set(0);
            deployed = false;
        }
    };
  }
        
 @Override
  public void periodic() {
    // SmartDashboard.putNumber("Motorspeed", IntakeExtendMotor.get);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
