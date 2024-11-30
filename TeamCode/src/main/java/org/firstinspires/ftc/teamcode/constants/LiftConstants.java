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
    public static int HORIZONTAL_LIFT_LOW_POSITION = 300;
    public static int HORIZONTAL_LIFT_MID_POSITION = 600;
    public static int HORIZONTAL_LIFT_HIGH_POSITION = 900;
    public static double HORIZONTAL_LIFT_POWER = 0.5;

    //PID for Vertical.
    public static double LIFT_KP = 0.009;  // Proportional gain
    public static double LIFT_KI = 0.0;   // Integral gain - if lift does not hold or reach position.
    public static double LIFT_KD = 0.004;   // Derivative gain - dampens power while approaching the set target.

    //PID threshold. PID is enabled only within a specified threshold distance from the target
    public static int REDUCE_POWER_THRESHOLD = 150;
}

