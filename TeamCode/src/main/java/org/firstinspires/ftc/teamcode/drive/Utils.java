package org.firstinspires.ftc.teamcode.drive;

public class Utils {
    private Utils(){}

    public static double cmToInch(double cm){
        return cm * 0.393701;
    }
    public static double tileToInch(double tiles){
        return cmToInch(60*tiles);
    }
}
