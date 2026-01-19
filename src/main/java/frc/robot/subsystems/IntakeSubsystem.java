// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.SparkFlex;
import com.revrobotics.SparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex m_intakeMotor = new SparkFlex(Constants.IntakeConstants.intakeMotorPort, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  public Command StartIntakeCommand() {
    return runOnce(
        () -> {
          m_intakeMotor.set(Constants.IntakeConstants.intakeMotorSpeedPercent / 100.0); // Set intake motor to full speed
        });
  }

  public Command StopIntakeCommand() {
    return runOnce(
        () -> {
          m_intakeMotor.set(0.0); // Set intake motor to full speed
        });
  }
}
