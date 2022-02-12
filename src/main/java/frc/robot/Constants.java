/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * Add your docs here.
 */
public class Constants {
    public static final class DriveConstants {

        public static final double kTrackwidthMeters = 0.612775;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = 0.68951;
        public static final double kvVoltSecondsPerMeter = 1.7743;
        public static final double kaVoltSecondsSquaredPerMeter = 0.44574;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.00031411;
    }

    public static final class OIConstants {

        public static final int kDriverControllerPort = 1;
        public static final int xboxControllerDriver = 0;
        public static final int xboxControllerAssist = 1;
        
        public static final int buttonA = 1;
        public static final int buttonB = 2;
        public static final int buttonX = 3;
        public static final int buttonY = 4;
        
        public static final int buttonBumperLeft = 5;
        public static final int buttonBumperRight = 6;
        
        public static final int buttonSelect = 7;
        public static final int buttonStart = 8;
        
        public static final int rightStickY = 5;
        public static final int leftStickY = 1;
        
        public static final int rightStickX = 4;
        public static final int leftStickX = 0;
        
        public static final int rightTrigger = 3;
        public static final int leftTrigger = 2;
    
        public static final int rightJoystickPush = 12;
        public static final int leftJoystickPush = 11;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;// default value 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;// default value 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class FalconDriveConstants {

        public static final int FrontRightTalon = 20; // PDP:13
        public static final int FrontLeftTalon = 23; // PDP:1
        public static final int BackRightTalon = 22; // PDP:14
        public static final int BackLeftTalon = 21; // PDP:13
        public static final int TopRightTalon = 24; // PDP:3
        public static final int TopLeftTalon = 25; // PDP:2

        public static final double wheelcircumference = 0.49149; // meters
        public static final double cpr = 2048; // count per rotation for talonFX integrated encoder
        private static final double gearRatio = 8.41; // to 1
        private static final double cpWheelr = cpr * gearRatio; // (17223.68) count per rotation for talonFX integrated
                                                                // encoder for each full wheel rotation
        public static final double distancePerPulse = wheelcircumference / cpWheelr;

    }

    public static final class LiftPositions {
        public static final int Top = 768000; // 752000 actual number;
        public static final int Hover = 375000;
        public static final int Increment = 192000;
        public static final int Home = 0;
    }

    public static final class TurrentConstants {
        public static final int TurretSparkMax = 2; 
    }

    public static final class ShooterConstants {
        public static final int ShooterSparkMax1 = 31; 
        public static final int ShooterSparkMax2 = 32;

    }

    public static final class IntakeConstants {
        public static final int IntakeElbowSparkMax = 30;
        public static final int IntakeWheelsSpark = 7;
    }

	public static final class RangeSensorConstants {
        public static final int TimeOfFlightLiftBottom = 88;
        public static final int TimeOfFlightLiftTop = 89;
    }

    public static final class IndexerConstants {
	    public static final int IndexerSparkMax = 3;
	}

	public static final class LiftConstants {
        public static final int LiftLeftSparkMax = 5;
        public static final int LiftRightSSparkMax = 3;
    }

}
