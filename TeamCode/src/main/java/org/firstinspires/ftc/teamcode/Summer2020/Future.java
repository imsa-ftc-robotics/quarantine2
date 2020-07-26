package org.firstinspires.ftc.teamcode.Summer2020;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public abstract class Future {
    abstract boolean poll();


    public static Future WaitMilliseconds(long milliseconds) {
        final long destination = System.nanoTime() + (milliseconds * 1000000);
        return new Future() {
            @Override boolean poll() {
                return System.nanoTime() >= destination;
            }
        };
    }
    public static Future WaitForMotor(final DcMotorEx motor) {
        return new Future() {
            @Override boolean poll() {
                return !motor.isBusy();
            }
        };
    }
    public static Future And(final Future a, final Future b) {
        return new Future() {
            @Override boolean poll() {
                return a.poll() && b.poll();
            }
        };
    }
    public static Future WhenPressed(final DigitalChannel touch_sensor) {
        return new Future() {
            @Override boolean poll() { return !touch_sensor.getState(); }
        };
    }
    public static Future Or(final Future a, final Future b) {
        return new Future() {
            @Override boolean poll() {
                return a.poll() || b.poll();
            }
        };
    }

    void wait_not_sleep() { while(!poll()) {} }

}
