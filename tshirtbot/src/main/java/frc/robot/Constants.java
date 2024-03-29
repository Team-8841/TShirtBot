// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {

        public static final int k_leftFrontMotorPort = 1;
        public static final int k_leftBackMotorPort = 2;
        public static final int k_rightFrontMotorPort = 3;
        public static final int k_rightBackMotorPort = 4;

        public static final int k_CurrentLimit = 60;
        public static final double k_DTRampRate = 0.5;
        public static final double k_MaxOutput = 1;

    }


    public static final class OIConstants{ 
        public static final int k_ControllerPort = 0;
    }


    public static final class TurretConstants {
        public static final int k_anglePort = 1;
        public static final int k_chargePort = 2;
        public static final int k_shootPort = 3;
    }
}


