package frc.robot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.TurretSubsystemConstants;

public class Configs {

 
    /* *****************
    * TurretSubsystem 
    */
   public static final class TurretSubsystem {
     public static final SparkMaxConfig turretConfig = new SparkMaxConfig();
 
     static {
       // Configure basic setting of the turret motor
       turretConfig
         .smartCurrentLimit(TurretSubsystemConstants.kTurretCurrentLimit)
         .closedLoopRampRate(TurretSubsystemConstants.kTurretRampRate);
 
       /*
        * Configure the closed loop controller. We want to make sure we set the
        * feedback sensor as the primary encoder.
        */
       turretConfig
           .closedLoop
           .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
           // Set PID values for position control. We don't need to pass a closed
           // loop slot, as it will default to slot 0.
           .pid(TurretSubsystemConstants.kTurretKp, 
               TurretSubsystemConstants.kTurretKi, 
               TurretSubsystemConstants.kTurretKd)
           .outputRange(-1,1)
           .maxMotion
           // Set MAXMotion parameters for position control
           .maxVelocity(2000)
           .maxAcceleration(10000)
           .allowedClosedLoopError(0.25);
 
       turretConfig.idleMode(IdleMode.kBrake);
     }
   }

  
}