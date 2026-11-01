package org.firstinspires.ftc.teamcode.Subsystems.Turret;

/**
 * Picks which hive cell the turret aims at, based on alliance and which quadrant of the
 * field the robot is in. Coordinates use the field frame: (0,0) bottom-left, X right, Y up, inches.
 *
 * Quadrant index = (robotY >= SPLIT_Y ? 2 : 0) + (robotX >= SPLIT_X ? 1 : 0)
 *   0 = bottom-left, 1 = bottom-right, 2 = top-left, 3 = top-right
 */
public class HiveTargeting {

    public enum Alliance { RED, BLUE }

    // ---------- Field split lines ----------
    public static double SPLIT_X = 72;
    public static double SPLIT_Y = 72;
    public static double SPLIT_HYSTERESIS = 4; // stops flicker when driving along a line

    // ---------- Goals per quadrant: {x, y}. Replace with exact manual values. ----------
    // Each alliance only shoots its own goals: upper cell when the robot is on the top
    // half of the field, lower cell when on the bottom half. Left/right doesn't matter.
    public static double[][] RED_GOALS = {
            {59, 61},  // 0 bottom-left  -> red lower
            {59, 61},  // 1 bottom-right -> red lower
            {59, 88},  // 2 top-left     -> red upper
            {59, 88}   // 3 top-right    -> red upper
    };
    public static double[][] BLUE_GOALS = {
            {86, 56},  // 0 bottom-left  -> blue lower
            {86, 56},  // 1 bottom-right -> blue lower
            {86, 85},  // 2 top-left     -> blue upper
            {86, 85}   // 3 top-right    -> blue upper
    };
    // -----------------------------------------------------------------------------------

    private boolean xHigh = false, yHigh = false;
    private int quadrant = 0;
    private double goalX, goalY;

    /** Call every loop. Updates the selected goal. */
    public void update(Alliance alliance, double robotX, double robotY) {
        if (xHigh) { if (robotX < SPLIT_X - SPLIT_HYSTERESIS) xHigh = false; }
        else       { if (robotX > SPLIT_X + SPLIT_HYSTERESIS) xHigh = true; }

        if (yHigh) { if (robotY < SPLIT_Y - SPLIT_HYSTERESIS) yHigh = false; }
        else       { if (robotY > SPLIT_Y + SPLIT_HYSTERESIS) yHigh = true; }

        quadrant = (yHigh ? 2 : 0) + (xHigh ? 1 : 0);
        double[][] goals = (alliance == Alliance.RED) ? RED_GOALS : BLUE_GOALS;
        goalX = goals[quadrant][0];
        goalY = goals[quadrant][1];
    }

    public double getGoalX()   { return goalX; }
    public double getGoalY()   { return goalY; }
    public int getQuadrant()   { return quadrant; }
}