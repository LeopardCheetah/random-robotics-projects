// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;



// this is a simple elevator that moves up and down :) 
public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  // assuming the elevator motor is run with a Falcon 500 as the motor
  private TalonFX m_elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorID);

  public Elevator() {}

  public void setElevatorSpeed(double speed){
    m_elevatorMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
