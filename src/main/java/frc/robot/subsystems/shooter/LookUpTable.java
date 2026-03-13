package frc.robot.subsystems.shooter;

public class LookUpTable {

	public record DataPoint(double distance, double rpm, double angle) {
	}

	public static DataPoint[] table = {
			new DataPoint(2.38, 3490, 10),
			new DataPoint(2.61, 3600, 10),
			new DataPoint(2.79, 3650, 10),
			new DataPoint(2.97, 3700, 10),
			new DataPoint(3.22, 3750, 11),
			new DataPoint(3.38, 3800, 11),
			new DataPoint(3.61, 3820, 11),
			new DataPoint(3.83, 3860, 11),
			new DataPoint(4.1, 3950, 11),
			new DataPoint(4.2, 3930, 11),
			new DataPoint(4.41, 4050, 11),
			new DataPoint(4.61, 4180, 11),
			new DataPoint(4.825, 4270, 11),
			new DataPoint(5.03, 4350, 11),
			new DataPoint(5.21, 4430, 11),
			new DataPoint(5.51, 4480, 11),
			new DataPoint(5.81, 4550, 11)
	};

	/**
	 * Gets an interpolated DataPoint from the table
	 * if the value is out of the table range, it will return the last value.
	 *
	 * @param distance the distance from the target
	 * @return a DataPoint with the RPM and hood angle
	 */
	public static DataPoint getPoint(double distance) {

		if (distance <= table[0].distance) {
			return table[0];
		} else if (distance >= table[table.length - 1].distance) {
			return table[table.length - 1];
		}

		for (int i = 0; i < table.length - 1; i++) {
			if (distance >= table[i].distance && distance <= table[i + 1].distance) {
				DataPoint a = table[i];
				DataPoint b = table[i + 1];
				return interpolatePoint(
						a,
						b,
						distance
				);
			}
		}
		return table[table.length - 1];
	}

	private static DataPoint interpolatePoint(DataPoint a, DataPoint b, double desiredDistance) {

		double t = (desiredDistance - a.distance) / (b.distance - a.distance);

		return new DataPoint(
				lerp(a.distance, b.distance, t),
				lerp(a.rpm, b.rpm, t),
				lerp(a.angle, b.angle, t)
		);
	}

	private static double lerp(double a, double b, double t) {
		return a + (b - a) * t;
	}
}