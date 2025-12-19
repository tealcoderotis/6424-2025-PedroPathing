package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterMath {
    private static final double ANGLE = Math.toRadians(53);
    private static final double GOAL_HEIGHT = 28.75;
    private static final double TOP_GOAL_HEIGHT = 53.75;
    private static final double EXIT_HEIGHT = 12;
    private static final double GOAL_RELATIVE_HEIGHT = GOAL_HEIGHT-EXIT_HEIGHT;
    private static final double TOP_GOAL_RELATIVE_HEIGHT = TOP_GOAL_HEIGHT-EXIT_HEIGHT;
    private static final double g = 386.1; //inches/s^2
    private final Telemetry telemetry;
    private double det(double x_1, double x_2, double y_1, double y_2) {
        //Finding DIST_MIN ends up needing lots of 2x2 determinants, this is easier to debug
        return x_1*y_2-x_2*y_1;
    }
    public double findLateralVelocity(Pose CurrentPose, int x_2, int y_2){
        double IntersectX; double IntersectY;
        double DIST_MAX = Math.sqrt(Math.pow((CurrentPose.getX()-x_2),2)+Math.pow((CurrentPose.getY()-y_2),2));
        double v_MAX = Math.sqrt((-g*DIST_MAX*DIST_MAX)/(2*(TOP_GOAL_RELATIVE_HEIGHT-DIST_MAX*Math.tan(ANGLE))*((Math.cos(ANGLE))*Math.cos(ANGLE))));
        //The distance to the closest part of the goal in the ball path is not simple, it needs an intersection as the goal is not just a single point
        //First Line
        double x_1 = CurrentPose.getX(); double y_1 = CurrentPose.getY();
        //Second Line depends on alliance
        double x_3; double y_3 = 144; double x_4; double y_4 = 120;
        if (x_2 > 72) {
            //Red Alliance
            x_3 = 120;
            x_4 = 132.75;
        } else {
            //Blue Alliance
            x_3 = 24;
            x_4 = 11.25;
        }
        //Cramer's Rule based formula found online for intersection
        double denominator = det(y_2-y_1, x_1-x_2, y_4-y_3, x_3-x_4);
        IntersectX = det(x_1*y_2-x_2*y_1, x_1-x_2, x_3*y_4-x_4*y_3, x_3-x_4)/denominator;
        IntersectY = det(y_2-y_1, x_1*y_2-x_2*y_1, y_4-y_3, x_3*y_4-x_4*y_3)/denominator;
        double DIST_MIN = Math.sqrt(Math.pow((CurrentPose.getX()-IntersectX),2)+Math.pow((CurrentPose.getY()-IntersectY),2));
        double v_MIN = Math.sqrt((-g*DIST_MIN*DIST_MIN)/(2*(GOAL_RELATIVE_HEIGHT-DIST_MIN*Math.tan(ANGLE))*((Math.cos(ANGLE))*Math.cos(ANGLE))));
        return (v_MAX+v_MIN)/2; //Velocity in inches/second, remember
        //Function output: (Hit top of goal speed+Hit lip of goal speed)/2
    }
    //REGRESSION PARAMETERS
    private static final double REGRESSION_A = 0.0000673187;
    private static final double REGRESSION_B = -0.0263884;
    public double ballVelocityToFlywheel(double BallV) {
        double d = (BallV*Math.cos(ANGLE)/g)*(BallV*Math.sin(ANGLE)+Math.sqrt(BallV*BallV*Math.sin(ANGLE)*Math.sin(ANGLE)+2*g*EXIT_HEIGHT));
        return (-1*REGRESSION_B+Math.sqrt(REGRESSION_B*REGRESSION_B+4*REGRESSION_A*d))/(2*REGRESSION_A);
    }
    public double angleFromGoal(Pose CurrentPose, int GoalX, int GoalY){
        telemetry.addData("GoalX", GoalX);
        telemetry.addData("GoalY", GoalY);
        double heading = CurrentPose.getHeading();
        double uncorrected = ((heading - Math.atan2((CurrentPose.getY()-GoalY),(CurrentPose.getX()-GoalX)))*180/Math.PI) % 360;
        telemetry.addData("uncorrected", uncorrected);
        if (Math.abs(uncorrected)>180) {
            return 360-uncorrected;
        } else {
            return uncorrected;
        }
    }

    public ShooterMath(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
}