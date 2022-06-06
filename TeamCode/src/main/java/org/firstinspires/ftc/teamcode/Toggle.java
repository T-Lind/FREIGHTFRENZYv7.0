package org.firstinspires.ftc.teamcode;

/**
 * Toggle Object - code to detect button presses and update a toggle variable in a fast loop
 * **/
public class Toggle {
    private boolean toggleState;
    private boolean pastInputState;

    /**
     * Writes the toggle state to a variable
     * Additionally the past state is as a default set to startingToggleState.
     * @param startingToggleState the state the toggle should initially start in
     */
    public Toggle(boolean startingToggleState) {
        toggleState = startingToggleState;
        pastInputState = startingToggleState;
    }

    /**
     * Get the value of the toggle
     * @return the value of the toggle
     */
    public boolean getToggleState(){
        return toggleState;
    }

    /**
     * updates the current state according to a leading edge detector
     * @param currentInputState what the input currently reads (ex. a button on the control pad)
     */
    public void updateLeadingEdge(boolean currentInputState) {
        // if the past input state was false and the current input state is true then
        if (pastInputState == false && currentInputState == true)
            // if the past toggle state was false then the current toggle state is true
            if (toggleState == false)
                toggleState = true;
                // otherwise the current toggle state is false
            else
                toggleState = false;

        // set the past state to the last read state
        pastInputState = currentInputState;
    }

    /**
     * updates the current state according to a falling edge detector
     * @param currentInputState what the input currently reads (ex. a button on the control pad)
     */
    public void updateFallingEdge(boolean currentInputState) {
        // if the past input state was false and the current input state is true then
        if (pastInputState == true && currentInputState == false)
            // if the past toggle state was false then the current toggle state is true
            if (toggleState == false)
                toggleState = true;
                // otherwise the current toggle state is false
            else
                toggleState = false;

        // set the past state to the last read state
        pastInputState = currentInputState;
    }
}
