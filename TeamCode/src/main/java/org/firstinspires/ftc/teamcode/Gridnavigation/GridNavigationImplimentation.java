package org.firstinspires.ftc.teamcode.Gridnavigation;

/**
 * Created by vasudevfamily on 8/31/17.
 */

public class GridNavigationImplimentation {

    double X1 = 0;
    //X1 is starting X coordinate
    double X2 = 2;
    //X2 is X destination
    double Y1 = 0;
    //Y1 is starting Y coordinate
    double Y2 = 4;
    //Y2 is Y destination
    double StartingAngle = 90;

    public static void main(String args[]) {

        GridNavigationImplimentation impl = new GridNavigationImplimentation();
        impl.GridNav();
        //  String returnValue = impl.add2(startValue);
        // System.out.println("return value in main: " + returnValue);
    }

    public void GridNav() {

        double tanAngle = 0;

        double turnAngle = 0;

        //L1 = X2 - X1
        double L1 = X2 - X1;

        //L2 = Y2 - Y1
        double L2 = Y2 - Y1;

        double Hypotenus = Math.hypot(L1, L2);

        double theta = Math.atan2(L2, L1);
        //atan2 automatically corrects for the limited domain of the inverse tangent function
        System.out.println(L1);
        System.out.println(L2);
        tanAngle = Math.toDegrees(theta);
        System.out.println("tanAngle is " + tanAngle);
        System.out.println("Hypotenus is " + Hypotenus);
        X1 = X2;
        System.out.println("X1 is " + X1);
        Y1 = Y2;
        System.out.println("Y1 is " + Y1);
        turnAngle = tanAngle - StartingAngle;
        System.out.println("Turn angle " + turnAngle);
        //if this doesn't work, comment out this entire if statement
        if(((turnAngle < 0) && (tanAngle < 0)) && (tanAngle < turnAngle)) {
            //StartingAngle = turnAngle - tanAngle;
            StartingAngle = tanAngle - turnAngle;
        }
        else if(((turnAngle < 0) && (tanAngle < 0)) && (turnAngle < tanAngle)){
            StartingAngle = turnAngle - tanAngle;
        }
        else if(((turnAngle > 0) && (tanAngle > 0)) && (turnAngle < tanAngle)){
            StartingAngle = tanAngle - turnAngle;
        }
        else if(((turnAngle > 0) && (tanAngle > 0)) && (tanAngle < turnAngle)) {
            StartingAngle = turnAngle - tanAngle;
        }
        else if(((turnAngle < 0) && (tanAngle > 0))) {
            StartingAngle = tanAngle - turnAngle;
        }
        else if(((turnAngle > 0) && (tanAngle < 0))) {
            StartingAngle = tanAngle - turnAngle;
        }
        else(){
            StartingAngle = turnAngle;
        }
        }
    }


          // Math.


/*
    public void angleCorrection(double L1, double L2, double tanAngle) {

        double turnAngle = 0;

        if((L1 < 0) && (L2 < 0)){
            turnAngle = tanAngle - StartingAngle + 180;
        }
        else if(L1 < 0){
            turnAngle = tanAngle - StartingAngle + 180;
        }
        else{
            turnAngle = tanAngle - StartingAngle;
        }

        System.out.println(turnAngle);

        StartingAngle = turnAngle;

        }

}
*/