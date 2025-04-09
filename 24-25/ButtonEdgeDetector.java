package org.firstinspires.ftc.teamcode;

//This constructor attempts to debounce buttons.
//I didn't write this, and while it doesn't look complicated
//i don't feel like figuring out how it works. Check Main
//TeleOp.java for usages.

public class ButtonEdgeDetector {
    boolean lastState;
    public ButtonEdgeDetector() {

    }
    public boolean updateActivate(boolean state) {
        if(state) {
            if(!lastState) {
                lastState = true;
                return true;
            } else {
                return false;
            }
        } else {
            lastState = false;
            return false;
        }
    }
}
