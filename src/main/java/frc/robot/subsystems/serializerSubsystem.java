// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkFlexConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class serializerSubsystem extends SubsystemBase {

SparkMax serialTubeMotor;
SparkMax serialSepMotor;
SparkMaxSim serialTubeMotorSim;
SparkMaxConfig serialTubeMotorConfig;
SparkMaxSim serialSepMotorSim;
SparkMaxConfig serialSepMotorConfig;


 boolean jam = true;
  /** Creates a new serializer. */
  public serializerSubsystem() {


    serialTubeMotor = new SparkMax(Constants.MotorConstants.kIntakeTubeMotorPort, MotorType.kBrushless);
    serialSepMotor = new SparkMax(Constants.MotorConstants.kIntakeTravellerMotorPort, MotorType.kBrushless);

   serialTubeMotorSim = new SparkMaxSim(serialTubeMotor, DCMotor.getNEO(1));
   SparkMaxConfig serialTubeMotorConfig = new SparkMaxConfig();
   serialTubeMotorConfig.idleMode(IdleMode.kBrake);
   serialTubeMotorConfig.smartCurrentLimit(20);

   serialSepMotorSim = new SparkMaxSim(serialSepMotor, DCMotor.getNEO(1));
   SparkMaxConfig serialSepMotorConfig = new SparkMaxConfig();
   serialSepMotorConfig.idleMode(IdleMode.kBrake);
   serialSepMotorConfig.smartCurrentLimit(20);

    serialTubeMotor.configure(serialTubeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    serialSepMotor.configure(serialSepMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

   public Command runQ() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
           if (!jam) {
          
          serialTubeMotor.set(-0.4);
          serialSepMotor.set(-0.4);

         } else {
          
          serialTubeMotor.set(0.7);
          serialSepMotor.set(0.7);
         }
        });
      };    

  public Command activateJam(){
    return runOnce(() -> { jam = !jam; });
  }

  public Command hardStopMotor(){
    return runOnce(() -> {
      serialTubeMotor.set(0.0);
      serialSepMotor.set(0.0);
    });
  }

  



  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("serializer sep. speed", (serialSepMotor.getAppliedOutput()));
    SmartDashboard.putNumber("serializer tube speed", (serialTubeMotor.getAppliedOutput()));

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    serialTubeMotorSim.iterate(serialTubeMotorSim.getVelocity(), 12, 0.02);
    serialSepMotorSim.iterate(serialSepMotorSim.getVelocity(), 12, 0.02);
  }

  public void setCurrentLimit(double limit) {
    serialTubeMotorConfig.smartCurrentLimit((int) limit);
    serialSepMotorConfig.smartCurrentLimit((int) limit);
    serialTubeMotor.configure(serialTubeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    serialSepMotor.configure(serialSepMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
