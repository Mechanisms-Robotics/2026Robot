package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.CAN;

public class LEDIOHW implements LEDIO {
    LEDIOInputs inputs = new LEDIOInputs();
    CAN can = new CAN(1);

    public void updateInputs(LEDIOInputs inputs) {
        // Update any inputs you need to log here
        this.inputs.LEDConnected = true;
    }
    
    public void sendMessage1() {
        can.writePacket(new byte[12], 0);
        this.inputs.lastLEDMessage = "Message1 sent!";
    }
}