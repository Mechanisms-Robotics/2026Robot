package frc.robot.subsystems.LED;

public interface LEDIO {
    public static class LEDIOInputs {
        // Add any inputs you need to log here
        public boolean LEDConnected = false;
        public String lastLEDMessage = null;
    }

    public default void updateInputs(LEDIOInputs inputs) {}
    
    public default void sendMessage1() {}
} 

