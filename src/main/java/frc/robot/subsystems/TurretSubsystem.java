// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretSubsystemConstants;
import frc.robot.Constants.TurretSubsystemConstants.TurretSubSystemSetpoints;

public class TurretSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum TurretSetpoints {
    kBase,
    kmaxSetpoint
  }
  
  /** Creates a new TurretSubsystem. */
  private SparkFlex turretMotor =
      new SparkFlex(44, MotorType.kBrushless);
  private SparkClosedLoopController turretController = turretMotor.getClosedLoopController();
  private RelativeEncoder turretEncoder = turretMotor.getEncoder();

  private double TurretCurrentTarget = TurretSubSystemSetpoints.kBase;
  
  public TurretSubsystem() {

    // Zero turret and elevator encoders on initialization
    turretEncoder.setPosition(0);
  }

  public void moveToSetpoint() {
    turretController.setReference(TurretCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  public void setTurretPower(double power) {
    turretMotor.set(power);
  }

    /**
   * Command to set the subsystem setpoint. This will set the Turret its predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(TurretSetpoints setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kmaxSetpoint:
              TurretCurrentTarget = TurretSubSystemSetpoints.kmaxTurretSetpoint;
              break;
            case kBase:
              TurretCurrentTarget = TurretSubSystemSetpoints.kBase;
              break;

          }
        });
  }


  /**
   * Command to run the turret motor. 
   * Intended to step through to adjust proper setpoints
   * When the command is interrupted, e.g. the button is released, the motor will stop.
   */
  public Command runTurretRightCommand() {
    return this.startEnd(
        () -> this.setTurretPower(TurretSubsystemConstants.TurretSetpointTestSpeed), 
        () -> this.setTurretPower(0.0));
  }

  public Command runTurretLeftCommand() {
    return this.startEnd(
        () -> this.setTurretPower((-1) * TurretSubsystemConstants.TurretSetpointTestSpeed), 
        () -> this.setTurretPower(0.0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
