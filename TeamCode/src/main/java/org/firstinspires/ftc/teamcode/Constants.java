package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.config.Config;


public class Constants {

    public static final class HardwareMapConstants {
        public static final String leftFront = "leftFront";
        public static final String rightFront = "rightFront";
        public static final String leftBack = "leftBack";
        public static final String rightBack = "rightBack";
    }
    public static double TICKS_PER_REV = 383.6;
    public static double WHEEL_DIAMETER = 0.1;
    public static double DISTANCE_PER_PULSE = WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;

    public static double leftFront_x = 0.381;
    public static double leftFront_y = 0.381;

    public static double rightFront_x = -0.381;
    public static double rightFront_y = 0.381;

    public static double leftBack_x = 0.381;
    public static double leftBack_y = -0.381;

    public static double rightBack_x = -0.381;
    public static double rightBack_y = -0.381;


    public static double TRACK_WIDTH = 0.4572;
    public static double CENTER_WHEEL_OFFSET = 0.0;

    public static double MAX_VELOCITY = 1.5;
    public static double MAX_ACCELERATION = 1.5;

    public static double B = 2.0;
    public static double ZETA = 0.7;



}
