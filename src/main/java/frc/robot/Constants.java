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
            public static class Drive {
                /**
                 * Static Friction Offset (to overcome the friction of the system)
                 */
                public static final double S = 0.0;
                /**
                 * Velocity Feedforward (to continue the current speed)
                 */
                public static final double V = 0.0;
                /**
                 * the voltage needed to reach a certain acceleration (i have no idea what number to put)
                 */
                public static final double A = 0.0;

                /**
                 * Proportional tuning - error
                 */
                public static final double P = 0.115;
                /**
                 * Integral tuning - learning
                 */
                public static final double I = 0.0;
                /**
                 * Derivative tuning - overshoot
                 */
                public static final double D = 0.0;
            } 

            public static class Steer {
                /**
                 * Static Friction Offset (to overcome the friction of the system)
                 */
                public static final double S = 0.0;
                /**
                 * Velocity Feedforward (to continue the current speed)
                 */
                public static final double V = 0.0;
                /**
                 * the voltage needed to reach a certain acceleration (i have no idea what number to put)
                 */
                public static final double A = 0.0;


                /**
                 * Proportional tuning - error
                 */
                public static final double P = -10.5;
                /**
                 * Integral tuning - learning
                 */
                public static final double I = 0.0;
                /**
                 * Derivative tuning - overshoot
                 */
                public static final double D = 0.0;
            } 

        }

        public static class Stats {
            public static final double MAX_VOLTAGE = 12.0;
            public static final double STATOR_CURRENT_LIMIT = 35.0;
            public static final double SUPPLY_CURRENT_LIMIT = 35.0;



            


            /**
             * The ratio between the Motor and the center wheel of the Swerve module (which the CANcoder lies on)
             */
            public static final double ROTOR_TO_SENSOR_RATIO = 8.14;

            public static final double DRIVE_WHEEL_RADIUS_INCHES = 2;
            public static final double DRIVE_WHEEL_RADIUS_METERS = Units.inchesToMeters(DRIVE_WHEEL_RADIUS_INCHES);
            
        
        }
    }

    public static class Drive {
        
        public static class Stats {
            /**
             * Distance between the center of the right wheels to the center of the left wheels (Meters)
             */
            public static final double TRACK_WIDTH_METERS = 85.5;

            /**+
             * Distance between the center of the back wheels to the center of the front wheels (Meters)
             */
            public static final double WHEELBASE_METERS = 85.5;

            /**
             * The current degree of the steer mechanism (At what degree does the drive wheel start)
             */
            public static final double FRONT_LEFT_MODULE_OFFSET_DEGREES = -102;
            /**
             * The current degree of the steer mechanism (At what degree does the drive wheel start)
             */
            public static final double FRONT_RIGHT_MODULE_OFFSET_DEGREES = -156;
            /**
             * The current degree of the steer mechanism (At what degree does the drive wheel start)
             */
            public static final double BACK_LEFT_MODULE_OFFSET_DEGREES = -20;
            /**
             * The current degree of the steer mechanism (At what degree does the drive wheel start)
             */
            public static final double BACK_RIGHT_MODULE_OFFSET_DEGREES = 166;

            public static final double MAX_VELOCITY_METERS_PER_SECOND = 2.39268;
            public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(TRACK_WIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

            public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics( // TODO needs to be configured with diffrent constants that has the modules position relative to the middle of the robot
            new Translation2d(TRACK_WIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0), // ++
            new Translation2d(TRACK_WIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0), // +-
            new Translation2d(-TRACK_WIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0), // -+
            new Translation2d(-TRACK_WIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0) // --
            );

        }

        public static class Motors {
            public static final int FRONT_LEFT_DRIVE_FALCON_CANID = 3;
            public static final int FRONT_LEFT_STEER_FALCON_CANID = 9;

            public static final int FRONT_RIGHT_DRIVE_FALCON_CANID = 5;
            public static final int FRONT_RIGHT_STEER_FALCON_CANID = 6;

            public static final int BACK_LEFT_DRIVE_FALCON_CANID = 4;
            public static final int BACK_LEFT_STEER_FALCON_CANID = 10;

            public static final int BACK_RIGHT_DRIVE_FALCON_CANID = 8;
            public static final int BACK_RIGHT_STEER_FALCON_CANID = 7;


        }

        public static class Encoders {
            // ? Only the steer encoder exists (seperate from the encoder inside of the Falcon 500 because of ratio problems between the wheels of the swerve modules)
            public static final int FRONT_LEFT_STEER_ENCODER_CANID = 19;
            public static final int FRONT_RIGHT_STEER_ENCODER_CANID = 20;
            public static final int BACK_LEFT_STEER_ENCODER_CANID = 21;
            public static final int BACK_RIGHT_STEER_ENCODER_CANID = 18;
        }

    }
    
    public static class Operator {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static class OI {
        public static final int XBOX_CONTROLLER_PORT = 0;
        public static final double XBOX_CONTROLLER_DRIFT = 0.1;
    }

    public static class Field {
        //TODO CHANGE THIS TO ACTUAL FIELD LENGTH FOR 2024
        public static final double FIELD_LENGTH = 0.0;
        public static final double FIELD_WIDTH = 0.0;
        public static final double APRIL_TAG_WIDTH = Units.inchesToMeters(6.0);
    }
    
}