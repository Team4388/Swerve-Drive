/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import frc4388.utility.Gains;
import frc4388.utility.LEDPatterns;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ArcadeDriveConstants {
        public static final int DRIVE_LEFT_FRONT_CAN_ID = 2;
        public static final int DRIVE_RIGHT_FRONT_CAN_ID = 4;
	    public static final int DRIVE_LEFT_BACK_CAN_ID = 3;
        public static final int DRIVE_RIGHT_BACK_CAN_ID = 5;
        
        public static final int DRIVE_PIGEON_ID = 6;

        public static final int SMARTDASHBOARD_UPDATE_FRAME = 2;
    }

    public static final class SwerveDriveConstants {
        public static final double ROTATION_SPEED = 0.1;
        public static final double WHEEL_SPEED = 0.1;
        public static final double WIDTH = 0;
        public static final double HEIGHT = 0;
        public static final int LEFT_FRONT_STEER_CAN_ID = 0;
        public static final int LEFT_FRONT_WHEEL_CAN_ID = 0;
        public static final int RIGHT_FRONT_STEER_CAN_ID = 0;
        public static final int RIGHT_FRONT_WHEEL_CAN_ID = 0;
        public static final int LEFT_BACK_STEER_CAN_ID = 0;
        public static final int LEFT_BACK_WHEEL_CAN_ID = 0;
        public static final int RIGHT_BACK_STEER_CAN_ID = 0;
        public static final int RIGHT_BACK_WHEEL_CAN_ID = 0;
        public static final int LEFT_FRONT_STEER_CAN_ENCODER_ID = 0;
        public static final int RIGHT_FRONT_STEER_CAN_ENCODER_ID = 0;
        public static final int LEFT_BACK_STEER_CAN_ENCODER_ID = 0;
        public static final int RIGHT_BACK_STEER_CAN_ENCODER_ID = 0;

        // swerve PID constants
        public static final int SWERVE_SLOT_IDX = 0;
        public static final int SWERVE_PID_LOOP_IDX = 1;
        public static final int SWERVE_TIMEOUT_MS = 30;
        public static final Gains SWERVE_GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0, 1.0);
    }
    public static final class LEDConstants {
        public static final int LED_SPARK_ID = 0;

        public static final LEDPatterns DEFAULT_PATTERN = LEDPatterns.FOREST_WAVES;
    }

    public static final class OIConstants {
        public static final int XBOX_DRIVER_ID = 0;
        public static final int XBOX_OPERATOR_ID = 1;
    }
}
