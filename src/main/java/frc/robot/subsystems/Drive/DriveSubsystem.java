// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.Swerve.SwerveModule;


public class DriveSubsystem extends SubsystemBase {
  // TODO https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithPathPlanner
  // TODO https://github.com/CrossTheRoadElec/SwerveDriveExample/blob/main/src/main/java/frc/robot/CTRSwerve/CTRSwerveModule.java
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  private final AHRS m_navX;
  private final SwerveModulePosition[] m_modulePositions;
  private final SwerveDriveOdometry m_odometry;
  private ChassisSpeeds m_swerveSpeeds;
  private Pose2d m_currentPose;




  public DriveSubsystem() {
    this.m_frontLeftModule = new SwerveModule(
      Drive.Motors.kFrontLeftDriveFalconCANID, 
      Drive.Motors.kFrontLeftSteerFalconCANID, 
      Drive.Encoders.kFrontLeftSteerEncoderCANID, 
      Drive.Stats.kFrontLeftModuleOffsetInDegrees
    );
    this.m_frontRightModule = new SwerveModule(
      Drive.Motors.kFrontRightDriveFalconCANID, 
      Drive.Motors.kFrontRightSteerFalconCANID, 
      Drive.Encoders.kFrontRightSteerEncoderCANID,
      Drive.Stats.kFrontRightModuleOffsetInDegrees
      );
    this.m_backLeftModule = new SwerveModule(
      Drive.Motors.kBackLeftDriveFalconCANID, 
      Drive.Motors.kBackLeftSteerFalconCANID, 
      Drive.Encoders.kBackLeftSteerEncoderCANID,
      Drive.Stats.kBackLeftModuleOffsetInDegrees
    );
    this.m_backRightModule = new SwerveModule(
      Drive.Motors.kBackRightDriveFalconCANID, 
      Drive.Motors.kBackRightSteerFalconCANID, 
      Drive.Encoders.kBackRightSteerEncoderCANID,
      Drive.Stats.kBackRightModuleOffsetInDegrees
    );


    m_navX = new AHRS();


    m_modulePositions = new SwerveModulePosition[]{ 
      // https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/blob/develop/examples/mk3-testchassis/src/main/java/com/swervedrivespecialties/examples/mk3testchassis/subsystems/DrivetrainSubsystem.java
      new SwerveModulePosition(m_frontLeftModule.getVelocityMetersPerSecond(), m_frontLeftModule.getSteerAngle()), 
      new SwerveModulePosition(m_frontRightModule.getVelocityMetersPerSecond(), m_frontRightModule.getSteerAngle()),
      new SwerveModulePosition(m_backLeftModule.getVelocityMetersPerSecond(), m_backLeftModule.getSteerAngle()), 
      new SwerveModulePosition(m_backRightModule.getVelocityMetersPerSecond(), m_backRightModule.getSteerAngle())
    };
       
    //TODO use swerve position estimator https://docs.wpilib.org/en/latest/docs/software/advanced-controls/state-space/state-space-pose-estimators.html


    m_odometry = new SwerveDriveOdometry(Drive.Stats.kinematics, Rotation2d.fromDegrees((double)m_navX.getFusedHeading()), m_modulePositions); 
  
    m_swerveSpeeds = new ChassisSpeeds(0, 0, 0);

    m_currentPose = m_odometry.getPoseMeters(); // TODO needs to take the position from vision 
  }
/**
 * Sets the state of all of the swerve modules
 * @param moduleState
 * WPILib's SwerveModuleState library
 */
  public void setModulesStates(SwerveModuleState[] moduleState) {
    m_frontLeftModule.setModuleState(moduleState[0]);
    m_frontRightModule.setModuleState(moduleState[1]);
    m_backLeftModule.setModuleState(moduleState[2]);
    m_backRightModule.setModuleState(moduleState[3]);
  }

  /**
   * Sets the Speed / Angle / Stats of all of the modules
   * @param xVelocityMps
   * The X velocity (Meters Per Second)
   * @param yVelocityMps
   * The Y velocity (Meters Per Second)
   * @param rotationVelocityRps
   * Rotation velocity (Radians Per Second)
   */
  public void setModules(double xVelocityMps, double yVelocityMps, double rotationVelocityRps) {
    // xVelocityMps=0;
    // yVelocityMps*=-1.0;
    // rotationVelocityRps = 0;
    this.m_swerveSpeeds = new ChassisSpeeds(-xVelocityMps, -yVelocityMps, -rotationVelocityRps);
    SwerveModuleState[] target_states = Drive.Stats.kinematics.toSwerveModuleStates(this.m_swerveSpeeds);
    setModulesStates(target_states);
  }

  public void print(){
    // System.out.println("\n");
    // System.out.println("FL MOTOR ANGLE " + this.m_frontLeftModule.getSteerEncoder().getAbsolutePosition().getValueAsDouble() / 360 + " degrees");
    // System.out.println("FR MOTOR ANGLE " + this.m_frontRightModule.getSteerEncoder().getAbsolutePosition().getValueAsDouble() / 360 + " degrees");
    // System.out.println("BR MOTOR ANGLE " + this.m_backRightModule.getSteerEncoder().getAbsolutePosition().getValueAsDouble()/ 360 + " degrees");
    // System.out.println("BL MOTOR ANGLE " + this.m_backLeftModule.getSteerEncoder().getAbsolutePosition().getValueAsDouble() / 360 + " degrees");
  }

  public void resetOdometry() {
      m_odometry.resetPosition(m_navX.getRotation2d(), m_modulePositions, m_currentPose);
  }

  public void setAllModulesToZero() {
    SwerveModuleState[] zeroStates = new SwerveModuleState[4];

    zeroStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(-Drive.Stats.kFrontLeftModuleOffsetInDegrees));
    zeroStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-Drive.Stats.kFrontRightModuleOffsetInDegrees));
    zeroStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-Drive.Stats.kBackLeftModuleOffsetInDegrees));
    zeroStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(-Drive.Stats.kBackRightModuleOffsetInDegrees));

    setModulesStates(zeroStates);
}

  /**
   * gets the angle of the navx 
   */
  public Rotation2d getGyroAngleInRotation2d() {
    return Rotation2d.fromDegrees((double)m_navX.getFusedHeading());
  }



  @Override
  public void periodic()
   {
    // module function that returns the position
    m_odometry.update(getGyroAngleInRotation2d(), new SwerveModulePosition[]{ 
      new SwerveModulePosition(m_frontLeftModule.getVelocityMetersPerSecond(), m_frontLeftModule.getSteerAngle()), 
      new SwerveModulePosition(m_frontRightModule.getVelocityMetersPerSecond(), m_frontRightModule.getSteerAngle()),
      new SwerveModulePosition(m_backLeftModule.getVelocityMetersPerSecond(), m_backLeftModule.getSteerAngle()), 
      new SwerveModulePosition(m_backRightModule.getVelocityMetersPerSecond(), m_backRightModule.getSteerAngle())
    });
    SwerveModuleState[] states = Drive.Stats.kinematics.toSwerveModuleStates(m_swerveSpeeds);

    m_frontLeftModule.setModuleState(states[0]);
    m_frontRightModule.setModuleState(states[1]);
    m_backLeftModule.setModuleState(states[2]);
    m_backRightModule.setModuleState(states[3]);

  }


// https://www.youtube.com/watch?v=mmNJjKJG8mw&ab_channel=LittletonRobotics
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
