package frc.robot.subsystems.led;

public enum LEDState {
	OFF(0),
	AUTO(1),
	SHOOT(2),
	INTAKE(3);

	public final int id;

	LEDState(int id) {
		this.id = id;
	}
}
