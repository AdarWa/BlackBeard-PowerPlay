package org.firstinspires.ftc.teamcode.util;

public class MathEx {

    public static double roundOff(double n, double digits){
        return Math.round(n*digits)/digits;
    }
}
