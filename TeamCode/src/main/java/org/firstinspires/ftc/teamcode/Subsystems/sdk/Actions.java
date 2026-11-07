package org.firstinspires.ftc.teamcode.Subsystems.sdk;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public final class Actions {

    private Actions() {
    }

    public static Action instant(Runnable runnable) {
        return new Action() {
            public void start() {
                runnable.run();
            }

            public boolean update() {
                return true;
            }

            public void stop() {
            }
        };
    }

    public static Action waitMs(long ms) {
        ElapsedTime timer = new ElapsedTime();
        return new Action() {
            public void start() {
                timer.reset();
            }

            public boolean update() {
                return timer.milliseconds() >= ms;
            }

            public void stop() {
            }
        };
    }

    public static Action waitUntil(BooleanSupplier condition, long timeoutMs) {
        ElapsedTime timer = new ElapsedTime();
        return new Action() {
            public void start() {
                timer.reset();
            }

            public boolean update() {
                return condition.getAsBoolean() || timer.milliseconds() >= timeoutMs;
            }

            public void stop() {
            }
        };
    }

    public static Action sequential(Action... actions) {
        return new Action() {
            private int index;
            private boolean started;

            public void start() {
                index = 0;
                started = false;
            }

            public boolean update() {
                while (index < actions.length) {
                    Action current = actions[index];
                    if (!started) {
                        current.start();
                        started = true;
                    }
                    if (!current.update()) {
                        return false;
                    }
                    index++;
                    started = false;
                }
                return true;
            }

            public void stop() {
                if (index < actions.length && started) {
                    actions[index].stop();
                }
            }
        };
    }

    public static Action parallel(Action... actions) {
        boolean[] done = new boolean[actions.length];
        return new Action() {
            public void start() {
                for (int i = 0; i < actions.length; i++) {
                    done[i] = false;
                    actions[i].start();
                }
            }

            public boolean update() {
                boolean all = true;
                for (int i = 0; i < actions.length; i++) {
                    if (!done[i]) {
                        done[i] = actions[i].update();
                    }
                    all &= done[i];
                }
                return all;
            }

            public void stop() {
                for (int i = 0; i < actions.length; i++) {
                    if (!done[i]) {
                        actions[i].stop();
                    }
                }
            }
        };
    }

    public static boolean run(LinearOpMode opMode, Action action, Runnable everyLoop) {
        action.start();
        boolean finished = false;
        while (opMode.opModeIsActive() && !finished) {
            everyLoop.run();
            finished = action.update();
        }
        if (!finished) {
            action.stop();
        }
        return finished;
    }
}
