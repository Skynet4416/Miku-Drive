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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;


public class DriveSubsystem extends SubsystemBase {
    // TODO https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithPathPlanner
    // TODO https://github.com/CrossTheRoadElec/SwerveDriveExample/blob/main/src/main/java/frc/robot/CTRSwerve/CTRSwerveModule.java
    private final SwerveModule mFrontLeftModule;
    private final SwerveModule mFrontRightModule;
    private final SwerveModule mBackLeftModule;
    private final SwerveModule mBackRightModule;
    private final ShuffleboardTab mSwerveModulesTab;
    private final ShuffleboardTab mDriveTab;
    private final AHRS mNavX;
    private final SwerveModulePosition[] mModulePositions;
    private final SwerveDriveOdometry mOdometry;
    private ChassisSpeeds mSwerveSpeeds;
    private Pose2d mCurrentPose;




    public DriveSubsystem() {
        this.mFrontLeftModule = new SwerveModule(
        Drive.Motors.FRONT_LEFT_DRIVE_FALCON_CANID, 
        Drive.Motors.FRONT_LEFT_STEER_FALCON_CANID, 
        Drive.Encoders.FRONT_LEFT_STEER_ENCODER_CANID, 
        Drive.Stats.FRONT_LEFT_MODULE_OFFSET_DEGREES
        );
        this.mFrontRightModule = new SwerveModule(
        Drive.Motors.FRONT_RIGHT_DRIVE_FALCON_CANID, 
        Drive.Motors.FRONT_RIGHT_STEER_FALCON_CANID, 
        Drive.Encoders.FRONT_RIGHT_STEER_ENCODER_CANID,
        Drive.Stats.FRONT_RIGHT_MODULE_OFFSET_DEGREES
        );
        this.mBackLeftModule = new SwerveModule(
        Drive.Motors.BACK_LEFT_DRIVE_FALCON_CANID, 
        Drive.Motors.BACK_LEFT_STEER_FALCON_CANID, 
        Drive.Encoders.BACK_LEFT_STEER_ENCODER_CANID,
        Drive.Stats.BACK_LEFT_MODULE_OFFSET_DEGREES
        );
        this.mBackRightModule = new SwerveModule(
        Drive.Motors.BACK_RIGHT_DRIVE_FALCON_CANID, 
        Drive.Motors.BACK_RIGHT_STEER_FALCON_CANID, 
        Drive.Encoders.BACK_RIGHT_STEER_ENCODER_CANID,
        Drive.Stats.BACK_RIGHT_MODULE_OFFSET_DEGREES
        );


        this.mNavX = new AHRS();


        this.mModulePositions = new SwerveModulePosition[]{ 
            // https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/blob/develop/examples/mk3-testchassis/src/main/java/com/swervedrivespecialties/examples/mk3testchassis/subsystems/DrivetrainSubsystem.java
            new SwerveModulePosition(mFrontLeftModule.getVelocityMetersPerSecond(), mFrontLeftModule.getSteerAngle()), 
            new SwerveModulePosition(mFrontRightModule.getVelocityMetersPerSecond(), mFrontRightModule.getSteerAngle()),
            new SwerveModulePosition(mBackLeftModule.getVelocityMetersPerSecond(), mBackLeftModule.getSteerAngle()), 
            new SwerveModulePosition(mBackRightModule.getVelocityMetersPerSecond(), mBackRightModule.getSteerAngle())
        };
        
        //TODO use swerve position estimator https://docs.wpilib.org/en/latest/docs/software/advanced-controls/state-space/state-space-pose-estimators.html


        this.mOdometry = new SwerveDriveOdometry(Drive.Stats.KINEMATICS, Rotation2d.fromDegrees((double)mNavX.getFusedHeading()), mModulePositions); 

        this.mSwerveSpeeds = new ChassisSpeeds(0, 0, 0);

        this.mCurrentPose = mOdometry.getPoseMeters(); // TODO needs to take the position from vision 
        this.mSwerveModulesTab = Shuffleboard.getTab("Swerve Modules");
        this.mDriveTab = Shuffleboard.getTab("Drive");
        setAllModulesToZero();
    }
    /**
     * Sets the state of all of the swerve modules
     * @param moduleState
     * WPILib's SwerveModuleState library
     */
    public void setModulesStates(SwerveModuleState[] moduleState) {
        mFrontLeftModule.setModuleState(moduleState[0]);
        mFrontRightModule.setModuleState(moduleState[1]);
        mBackLeftModule.setModuleState(moduleState[2]);
        mBackRightModule.setModuleState(moduleState[3]);
    }

    public double getXVelocityFieldOriented(double targetVelocityX, double targetVelocityY) {
        double offsetAngle = getGyroAngleInRotation2d().getDegrees() - Drive.Stats.FIELD_HEADING_OFFSET;
        return targetVelocityX * Math.cos(Units.degreesToRadians(offsetAngle)) - targetVelocityY * Math.sin(Units.degreesToRadians(offsetAngle));
    }

    public double getYVelocityFieldOriented(double targetVelocityX, double targetVelocityY) {
        double offsetAngle = getGyroAngleInRotation2d().getDegrees() - Drive.Stats.FIELD_HEADING_OFFSET;
        return targetVelocityX * Math.sin(Units.degreesToRadians(offsetAngle)) + targetVelocityY * Math.cos(Units.degreesToRadians(offsetAngle));
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
        this.mSwerveSpeeds = new ChassisSpeeds(
            -getXVelocityFieldOriented(xVelocityMps, yVelocityMps), 
            -getYVelocityFieldOriented(xVelocityMps, yVelocityMps), 
            -rotationVelocityRps
        );
        SwerveModuleState[] targetStates = Drive.Stats.KINEMATICS.toSwerveModuleStates(this.mSwerveSpeeds);
        setModulesStates(targetStates);
    }

    public void resetOdometry() {
        mOdometry.resetPosition(mNavX.getRotation2d(), mModulePositions, mCurrentPose);
    }

    public void setAllModulesToZero() {
        SwerveModuleState[] zeroStates = new SwerveModuleState[4];

        zeroStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(-Drive.Stats.FRONT_LEFT_MODULE_OFFSET_DEGREES));
        zeroStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-Drive.Stats.FRONT_RIGHT_MODULE_OFFSET_DEGREES));
        zeroStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-Drive.Stats.BACK_LEFT_MODULE_OFFSET_DEGREES));
        zeroStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(-Drive.Stats.BACK_RIGHT_MODULE_OFFSET_DEGREES));

        setModulesStates(zeroStates);
    }

    /**
     * gets the angle of the navx 
     */
    public Rotation2d getGyroAngleInRotation2d() {
        return Rotation2d.fromDegrees((double)mNavX.getFusedHeading());
    }



    public void addValuesForModule(SwerveModule module, String name) {
        mSwerveModulesTab.addDouble(name + " Drive Velocity (Meters Per Second):", (() -> module.getVelocityMetersPerSecond()));
        mSwerveModulesTab.addDouble(name + " Steer Position (Degrees):", (() -> module.getSteerAngle().getDegrees()));
        mSwerveModulesTab.addDouble(name + " Steer Position (Rotations):", (() -> module.getSteerAngle().getRotations()));
        mSwerveModulesTab.addDouble(name + " Encoder Absolute Position (Rotations):", (() -> module.getSteerEncoder().getAbsolutePosition().getValue()));
    }

    @Override
    public void periodic()
    {
        // module function that returns the position
        mOdometry.update(getGyroAngleInRotation2d(), new SwerveModulePosition[]{ 
        new SwerveModulePosition(mFrontLeftModule.getVelocityMetersPerSecond(), mFrontLeftModule.getSteerAngle()), 
        new SwerveModulePosition(mFrontRightModule.getVelocityMetersPerSecond(), mFrontRightModule.getSteerAngle()),
        new SwerveModulePosition(mBackLeftModule.getVelocityMetersPerSecond(), mBackLeftModule.getSteerAngle()), 
        new SwerveModulePosition(mBackRightModule.getVelocityMetersPerSecond(), mBackRightModule.getSteerAngle())
        });
        SwerveModuleState[] states = Drive.Stats.KINEMATICS.toSwerveModuleStates(mSwerveSpeeds);
        setModulesStates(states);
        addValuesForModule(mFrontLeftModule, "Front Left Module");
        addValuesForModule(mFrontRightModule, "Front Right Module");
        addValuesForModule(mBackLeftModule, "Back Left Module");
        addValuesForModule(mBackRightModule, "Back Right Module");
        mDriveTab.addDouble("NavX Angle (Degrees)", (() -> getGyroAngleInRotation2d().getDegrees()));
    }


    // https://www.youtube.com/watch?v=mmNJjKJG8mw&ab_channel=LittletonRobotics
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
