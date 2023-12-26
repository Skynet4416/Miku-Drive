// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class Swerve {

    public static class PID {
      // ! --- DO NOT USE THESE PID k VARIABLES IN PRODUCTION! I DID NOT TEST THEM YET ------------------ 
      public static class Drive {
        /**
         * Static Friction Offset (to overcome the friction of the system)
         */
        public static final double kS = 0.0;
        /**
         * Velocity Feedforward (to continue the current speed)
         */
        public static final double kV = 0.0;
        /**
         * the voltage needed to reach a certain acceleration (i have no idea what number to put)
         */
        public static final double kA = 0.0;

        /**
         * Proportional tuning - error
         */
        public static final double kP = 0.115;
        /**
         * Integral tuning - learning
         */
        public static final double kI = 0.0;
        /**
         * Derivative tuning - overshoot
         */
        public static final double kD = 0.0;
      } 

      public static class Steer {
        /**
         * Static Friction Offset (to overcome the friction of the system)
         */
        public static final double kS = 0.0;
        /**
         * Velocity Feedforward (to continue the current speed)
         */
        public static final double kV = 0.0;
        /**
         * the voltage needed to reach a certain acceleration (i have no idea what number to put)
         */
        public static final double kA = 0.0;


        /**
         * Proportional tuning - error
         */
        public static final double kP = -10.5;
        /**
         * Integral tuning - learning
         */
        public static final double kI = 0.0;
        /**
         * Derivative tuning - overshoot
         */
        public static final double kD = 0.0;
      } 

    }

    public static class Stats {
      public static final double kMaxVoltage = 12.0;
      public static final double kStatorCurrentLimit = 35.0;
      public static final double kSupplyCurrentLimit = 35.0;
      
      /**
       * Distance between the center of the right wheels to the center of the left wheels (Meters)
       */
      public static final double kTrackWidthMeters = 85.5;

      /**+
       * Distance between the center of the back wheels to the center of the front wheels (Meters)
       */
      public static final double kWheelbaseMeters = 85.5;

      


      /**
       * The ratio between the Motor and the center wheel of the Swerve module (which the CANcoder lies on)
       */
      public static final double kRotorToSensorRatio = 8.14;

      public static final double kDriveWheelRadiusInches = 2;
      public static final double wheelRadiusMeters = Units.inchesToMeters(kDriveWheelRadiusInches);
      
      
    }
  }

  public static class Drive {
      
    public static class Stats {
        /**
         * Distance between the center of the right wheels to the center of the left wheels (Meters)
         */
        public static final double kTrackWidthMeters = 85.5;

        /**
         * Distance between the center of the back wheels to the center of the front wheels (Meters)
         */
        public static final double kWheelbaseMeters = 85.5;

        /**
         * The current degree of the steer mechanism (At what degree does the drive wheel start)
         */
        public static final double kFrontLeftModuleOffsetInDegrees = -102;
        /**
         * The current degree of the steer mechanism (At what degree does the drive wheel start)
         */
        public static final double kFrontRightModuleOffsetInDegrees = -156;
        /**
         * The current degree of the steer mechanism (At what degree does the drive wheel start)
         */
        public static final double kBackLeftModuleOffsetInDegrees = -20;
        /**
         * The current degree of the steer mechanism (At what degree does the drive wheel start)
         */
        public static final double kBackRightModuleOffsetInDegrees = 166;

        public static final double kMaxVelocityMetersPerSecond = 2.39268;
        public static final double kMaxAngularVelocityRadiansPerSecond = kMaxVelocityMetersPerSecond /
        Math.hypot(kTrackWidthMeters / 2.0, kWheelbaseMeters / 2.0);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics( // TODO needs to be configured with diffrent constants that has the modules position relative to the middle of the robot
          new Translation2d(kTrackWidthMeters / 2.0, kWheelbaseMeters / 2.0), // ++
          new Translation2d(kTrackWidthMeters / 2.0, -kWheelbaseMeters / 2.0), // +-
          new Translation2d(-kTrackWidthMeters / 2.0, kWheelbaseMeters / 2.0), // -+
          new Translation2d(-kTrackWidthMeters / 2.0, -kWheelbaseMeters / 2.0) // --
        );

    }

    public static class Motors {
      public static final int kFrontLeftDriveFalconCANID = 3;
      public static final int kFrontLeftSteerFalconCANID = 9;

      public static final int kFrontRightDriveFalconCANID = 5;
      public static final int kFrontRightSteerFalconCANID = 6;

      public static final int kBackLeftDriveFalconCANID = 4;
      public static final int kBackLeftSteerFalconCANID = 10;

      public static final int kBackRightDriveFalconCANID = 8;
      public static final int kBackRightSteerFalconCANID = 7;


    }

    public static class Encoders {
      // ? Only the steer encoder exists (seperate from the encoder inside of the Falcon 500 because of ratio problems between the wheels of the swerve modules)
      public static final int kFrontLeftSteerEncoderCANID = 19;
      public static final int kFrontRightSteerEncoderCANID = 20;
      public static final int kBackLeftSteerEncoderCANID = 21;
      public static final int kBackRightSteerEncoderCANID = 18;
    }

  }
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class OI
  {
    public static final int kXboxControllerPort = 0;
    public static final double kXboxcontrollerDrift = 0.1;
  }
  public static class FieldConstants {
    //TODO CHANGE THIS TO ACTUAL FIELD LENGTH FOR 2024
    public static final double fieldLength = 0.0;
    public static final double fieldWidth = 0.0;
    public static final double aprilTagWidth = Units.inchesToMeters(6.0);
  }
  
}