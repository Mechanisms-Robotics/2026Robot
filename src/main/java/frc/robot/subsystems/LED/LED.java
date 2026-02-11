package frc.robot.subsystems.LED;

public class LED {

    private final LEDIO ledIO;
    private final LEDIO.LEDIOInputs inputs = new LEDIO.LEDIOInputs();

    public LED(LEDIO ledio) {
        this.ledIO = ledio;
    }

    public void updateInputs(LEDIO.LEDIOInputs inputs) {
        this.ledIO.updateInputs(inputs);
    }
    
    public void sendMessage1() {
        System.out.println("Message1 sent!");
        this.ledIO.sendMessage1();
    }
}
