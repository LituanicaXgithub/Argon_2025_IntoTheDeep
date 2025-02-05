package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

// 'Config' annotation is for Dashboard edit.
@Config
public class LiftConstants {

    //Vertical lift
    public static int LOW_POSITION = 100;
    public static int MID_POSITION = 850;
    public static int HIGH_POSITION = 1750;
    public static double LIFT_POWER = 1.0;

    //Horizontal lift (intake)

    public static int HORIZONTAL_LIFT_START = 0;
    public static int HORIZONTAL_LIFT_MAX_IN = 600;
    public static int HORIZONTAL_LIFT_MAX_OUT = 1400;
    public static double HORIZONTAL_LIFT_POWER = 0.5;

    //PID for Vertical.
    // PID coefficients for upward motion (heavier load)
    public static final double LIFT_UP_KP = 0.005;
    public static final double LIFT_UP_KI = 0.0001;
    public static final double LIFT_UP_KD = 0.001;

    // PID coefficients for downward motion (faster, less load)
    public static final double LIFT_DOWN_KP = 0.003;
    public static final double LIFT_DOWN_KI = 0.00005;
    public static final double LIFT_DOWN_KD = 0.0005;



    //PID threshold. PID is enabled only within a specified threshold distance from the target
    public static int REDUCE_POWER_THRESHOLD = 150;
}

