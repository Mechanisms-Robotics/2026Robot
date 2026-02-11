package frc.robot.subsystems.LED;

public class LED {

    private final LEDIO ledIO;
    private final LEDIO.LEDIOInputs inputs = new LEDIO.LEDIOInputs();

    public LED(LEDIO ledio) {
        this.ledIO = ledio;
    }

    public void sendMessage1() {
        this.ledIO.sendMessage1();
    }
}
