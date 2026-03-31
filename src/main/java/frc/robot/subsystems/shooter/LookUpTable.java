package frc.robot.subsystems.shooter;

public class LookUpTable {

	public record DataPoint(double distance, double rpm, double angle) {
	}

	public static DataPoint[] table = {
			new DataPoint(2.38, 5990, 10),
			new DataPoint(2.61, 6100, 10),
			new DataPoint(2.79, 6150, 10),
			new DataPoint(2.97, 6200, 10),
			new DataPoint(3.22, 6250, 10.6),
			new DataPoint(3.38, 6300, 10.6),
			new DataPoint(3.61, 6320, 10.6),
			new DataPoint(3.83, 6360, 10.6),
			new DataPoint(4.1, 6450, 10.6),
			new DataPoint(4.2, 6430, 11),
			new DataPoint(4.41, 6550, 11),
			new DataPoint(4.61, 6680, 11),
			new DataPoint(4.825, 6770, 11),
			new DataPoint(5.03, 6850, 11),
			new DataPoint(5.21, 6930, 11),
			new DataPoint(5.51, 6980, 11),
			new DataPoint(5.81, 7050, 11)
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