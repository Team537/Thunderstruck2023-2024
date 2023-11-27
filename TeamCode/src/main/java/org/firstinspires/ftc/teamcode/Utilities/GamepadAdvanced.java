package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadAdvanced {

    Gamepad gamepad;

    public boolean a;
    public boolean b;
    public boolean back;
    public boolean dpad_down;
    public boolean dpad_left;
    public boolean dpad_right;
    public boolean dpad_up;
    public boolean guide;
    public int id;
    public boolean left_bumper;
    public boolean left_stick_button;
    public float left_stick_x;
    public float left_stick_y;
    public float left_trigger;
    public boolean right_bumper;
    public boolean right_stick_button;
    public float right_stick_x;
    public float right_stick_y;
    public float right_trigger;
    public boolean start;
    public long timestamp;
    public boolean x;
    public boolean y;

    public boolean left_trigger_press;
    public boolean left_trigger_full_press;

    public boolean right_trigger_press;
    public boolean right_trigger_full_press;

    //a rising and falling edge
    public boolean a_rising_edge;
    public boolean a_falling_edge;
    boolean a_toggle;

    //b rising and falling edge
    public boolean b_rising_edge;
    public boolean b_falling_edge;
    boolean b_toggle;

    //back rising and falling edge
    public boolean back_rising_edge;
    public boolean back_falling_edge;
    public boolean back_toggle;

    //dpad_down rising and falling edge
    public boolean dpad_down_rising_edge;
    public boolean dpad_down_falling_edge;
    boolean dpad_down_toggle;

    // dpad_left rising and falling edge
    public boolean dpad_left_rising_edge;
    public boolean dpad_left_falling_edge;
    boolean dpad_left_toggle;

    // dpad_right rising and falling edge
    public boolean dpad_right_rising_edge;
    public boolean dpad_right_falling_edge;
    boolean dpad_right_toggle;

    // dpad_up rising and falling edge
    public boolean dpad_up_rising_edge;
    public boolean dpad_up_falling_edge;
    boolean dpad_up_toggle;

    //guide rising and falling edge
    public boolean guide_rising_edge;
    public boolean guide_falling_edge;
    boolean guide_toggle;

    // left_bumper rising and falling edge
    public boolean left_bumper_rising_edge;
    public boolean left_bumper_falling_edge;
    boolean left_bumper_toggle;

    // left_stick_button rising and falling edge
    public boolean left_stick_button_rising_edge;
    public boolean left_stick_button_falling_edge;
    boolean left_stick_button_toggle;

    // left_trigger_press rising and falling edge
    public boolean left_trigger_press_rising_edge;
    public boolean left_trigger_press_falling_edge;
    boolean left_trigger_press_toggle;

    // left_trigger_full_press rising and falling edge
    public boolean left_trigger_full_press_rising_edge;
    public boolean left_trigger_full_press_falling_edge;
    boolean left_trigger_full_press_toggle;

    // right_bumper rising and falling edge
    public boolean right_bumper_rising_edge;
    public boolean right_bumper_falling_edge;
    boolean right_bumper_toggle;

    // right_stick_button rising and falling edge
    public boolean right_stick_button_rising_edge;
    public boolean right_stick_button_falling_edge;
    boolean right_stick_button_toggle;

    // right_trigger_press rising and falling edge
    public boolean right_trigger_press_rising_edge;
    public boolean right_trigger_press_falling_edge;
    boolean right_trigger_press_toggle;

    // right_trigger_full_press rising and falling edge
    public boolean right_trigger_full_press_rising_edge;
    public boolean right_trigger_full_press_falling_edge;
    boolean right_trigger_full_press_toggle;

    // start rising and falling edge
    public boolean start_rising_edge;
    public boolean start_falling_edge;
    boolean start_toggle;

    // x rising and falling edge
    public boolean x_rising_edge;
    public boolean x_falling_edge;
    boolean x_toggle;

    // y rising and falling edge
    public boolean y_rising_edge;
    public boolean y_falling_edge;
    boolean y_toggle;

    Vector left_stick;
    float left_stick_magnitude;
    float left_stick_angle;

    Vector right_stick;
    float right_stick_magnitude;
    float right_stick_angle;

    public GamepadAdvanced(Gamepad gamepad) {
        this.gamepad = gamepad;
        initialize();
    }

    public void initialize() {

        dpad_down = gamepad.dpad_down;
        dpad_left = gamepad.dpad_left;
        dpad_right = gamepad.dpad_right;
        dpad_up = gamepad.dpad_up;
        guide = gamepad.guide;
        id = gamepad.id;
        left_bumper = gamepad.left_bumper;
        left_stick_button = gamepad.left_stick_button;
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = -gamepad.left_stick_y;
        left_trigger = gamepad.left_trigger;
        right_bumper = gamepad.right_bumper;
        right_stick_button = gamepad.right_stick_button;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = -gamepad.right_stick_y;
        right_trigger = gamepad.right_trigger;
        start = gamepad.start;
        timestamp = gamepad.timestamp;
        x = gamepad.x;
        y = gamepad.y;

        left_trigger_press = left_trigger > 0;
        left_trigger_full_press = left_trigger == 1;

        right_trigger_press = right_trigger > 0;
        right_trigger_full_press = right_trigger == 1;

        left_stick = new Vector(left_stick_x,left_stick_y);
        left_stick_magnitude = (float) new Vector(left_stick_x,left_stick_y).magnitude();
        left_stick_angle = (float) new Vector(left_stick_x,left_stick_y).angle();

        right_stick = new Vector(right_stick_x,right_stick_y);
        right_stick_magnitude = (float) new Vector(right_stick_x,right_stick_y).magnitude();
        right_stick_angle = (float) new Vector(right_stick_x,right_stick_y).angle();

        a_toggle = a;
        b_toggle = b;
        back_toggle = back;
        dpad_down_toggle = dpad_down;
        dpad_left_toggle = dpad_left;
        dpad_right_toggle = dpad_right;
        dpad_up_toggle = dpad_up;
        guide_toggle = guide;
        left_bumper_toggle = left_bumper;
        left_stick_button_toggle = left_stick_button;
        left_trigger_press_toggle = left_trigger_press;
        left_trigger_full_press_toggle = left_trigger_full_press;
        right_bumper_toggle = right_bumper;
        right_stick_button_toggle = right_stick_button;
        right_trigger_press_toggle = right_trigger_press;
        right_trigger_full_press_toggle = right_trigger_full_press;
        start_toggle = start;
        x_toggle = x;
        y_toggle = y;

    }

    public void tick() {

        dpad_down = gamepad.dpad_down;
        dpad_left = gamepad.dpad_left;
        dpad_right = gamepad.dpad_right;
        dpad_up = gamepad.dpad_up;
        guide = gamepad.guide;
        id = gamepad.id;
        left_bumper = gamepad.left_bumper;
        left_stick_button = gamepad.left_stick_button;
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = -gamepad.left_stick_y;
        left_trigger = gamepad.left_trigger;
        right_bumper = gamepad.right_bumper;
        right_stick_button = gamepad.right_stick_button;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = -gamepad.right_stick_y;
        right_trigger = gamepad.right_trigger;
        start = gamepad.start;
        timestamp = gamepad.timestamp;
        x = gamepad.x;
        y = gamepad.y;

        left_trigger_press = left_trigger > 0;
        left_trigger_full_press = left_trigger == 1;

        right_trigger_press = right_trigger > 0;
        right_trigger_full_press = right_trigger == 1;

        left_stick = new Vector(left_stick_x,left_stick_y);
        left_stick_magnitude = (float) new Vector(left_stick_x,left_stick_y).magnitude();
        left_stick_angle = (float) new Vector(left_stick_x,left_stick_y).angle();

        right_stick = new Vector(right_stick_x,right_stick_y);
        right_stick_magnitude = (float) new Vector(right_stick_x,right_stick_y).magnitude();
        right_stick_angle = (float) new Vector(right_stick_x,right_stick_y).angle();

        if (a == a_toggle) {
            a_rising_edge = false;
            a_falling_edge = false;
        } else {
            if (a) {
                a_rising_edge = true;
            } else {
                a_falling_edge = true;
            }
        }

        if (b == b_toggle) {
            b_rising_edge = false;
            b_falling_edge = false;
        } else {
            if (b) {
                b_rising_edge = true;
            } else {
                b_falling_edge = true;
            }
        }

        if (back == back_toggle) {
            back_rising_edge = false;
            back_falling_edge = false;
        } else {
            if (back) {
                back_rising_edge = true;
            } else {
                back_falling_edge = true;
            }
        }

        if (dpad_down == dpad_down_toggle) {
            dpad_down_rising_edge = false;
            dpad_down_falling_edge = false;
        } else {
            if (dpad_down) {
                dpad_down_rising_edge = true;
            } else {
                dpad_down_falling_edge = true;
            }
        }

        if (dpad_left == dpad_left_toggle) {
            dpad_left_rising_edge = false;
            dpad_left_falling_edge = false;
        } else {
            if (dpad_left) {
                dpad_left_rising_edge = true;
            } else {
                dpad_left_falling_edge = true;
            }
        }

        if (dpad_right == dpad_right_toggle) {
            dpad_right_rising_edge = false;
            dpad_right_falling_edge = false;
        } else {
            if (dpad_right) {
                dpad_right_rising_edge = true;
            } else {
                dpad_right_falling_edge = true;
            }
        }

        if (dpad_up == dpad_up_toggle) {
            dpad_up_rising_edge = false;
            dpad_up_falling_edge = false;
        } else {
            if (dpad_up) {
                dpad_up_rising_edge = true;
            } else {
                dpad_up_falling_edge = true;
            }
        }

        if (guide == guide_toggle) {
            guide_rising_edge = false;
            guide_falling_edge = false;
        } else {
            if (guide) {
                guide_rising_edge = true;
            } else {
                guide_falling_edge = true;
            }
        }

        if (left_bumper == left_bumper_toggle) {
            left_bumper_rising_edge = false;
            left_bumper_falling_edge = false;
        } else {
            if (left_bumper) {
                left_bumper_rising_edge = true;
            } else {
                left_bumper_falling_edge = true;
            }
        }

        if (left_stick_button == left_stick_button_toggle) {
            left_stick_button_rising_edge = false;
            left_stick_button_falling_edge = false;
        } else {
            if (left_stick_button) {
                left_stick_button_rising_edge = true;
            } else {
                left_stick_button_falling_edge = true;
            }
        }

        if (left_trigger_press == left_trigger_press_toggle) {
            left_trigger_press_rising_edge = false;
            left_trigger_press_falling_edge = false;
        } else {
            if (left_trigger_press) {
                left_trigger_press_rising_edge = true;
            } else {
                left_trigger_press_falling_edge = true;
            }
        }

        if (left_trigger_full_press == left_trigger_full_press_toggle) {
            left_trigger_full_press_rising_edge = false;
            left_trigger_full_press_falling_edge = false;
        } else {
            if (left_trigger_full_press) {
                left_trigger_full_press_rising_edge = true;
            } else {
                left_trigger_full_press_falling_edge = true;
            }
        }

        if (right_bumper == right_bumper_toggle) {
            right_bumper_rising_edge = false;
            right_bumper_falling_edge = false;
        } else {
            if (right_bumper) {
                right_bumper_rising_edge = true;
            } else {
                right_bumper_falling_edge = true;
            }
        }

        if (right_stick_button == right_stick_button_toggle) {
            right_stick_button_rising_edge = false;
            right_stick_button_falling_edge = false;
        } else {
            if (right_stick_button) {
                right_stick_button_rising_edge = true;
            } else {
                right_stick_button_falling_edge = true;
            }
        }

        if (right_trigger_press == right_trigger_press_toggle) {
            right_trigger_press_rising_edge = false;
            right_trigger_press_falling_edge = false;
        } else {
            if (right_trigger_press) {
                right_trigger_press_rising_edge = true;
            } else {
                right_trigger_press_falling_edge = true;
            }
        }

        if (right_trigger_full_press == right_trigger_full_press_toggle) {
            right_trigger_full_press_rising_edge = false;
            right_trigger_full_press_falling_edge = false;
        } else {
            if (right_trigger_full_press) {
                right_trigger_full_press_rising_edge = true;
            } else {
                right_trigger_full_press_falling_edge = true;
            }
        }

        if (start == start_toggle) {
            start_rising_edge = false;
            start_falling_edge = false;
        } else {
            if (start) {
                start_rising_edge = true;
            } else {
                start_falling_edge = true;
            }
        }

        if (x == x_toggle) {
            x_rising_edge = false;
            x_falling_edge = false;
        } else {
            if (x) {
                x_rising_edge = true;
            } else {
                x_falling_edge = true;
            }
        }

        if (y == y_toggle) {
            y_rising_edge = false;
            y_falling_edge = false;
        } else {
            if (y) {
                y_rising_edge = true;
            } else {
                y_falling_edge = true;
            }
        }

        a_toggle = a;
        b_toggle = b;
        back_toggle = back;
        dpad_down_toggle = dpad_down;
        dpad_left_toggle = dpad_left;
        dpad_right_toggle = dpad_right;
        dpad_up_toggle = dpad_up;
        guide_toggle = guide;
        left_bumper_toggle = left_bumper;
        left_stick_button_toggle = left_stick_button;
        left_trigger_press_toggle = left_trigger_press;
        left_trigger_full_press_toggle = left_trigger_full_press;
        right_bumper_toggle = right_bumper;
        right_stick_button_toggle = right_stick_button;
        right_trigger_press_toggle = right_trigger_press;
        right_trigger_full_press_toggle = right_trigger_full_press;
        start_toggle = start;
        x_toggle = x;
        y_toggle = y;

    }


}
