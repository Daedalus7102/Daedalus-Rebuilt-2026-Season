package frc.robot.subsystems.shooter;

public class LookUpTable {
    //array anlgulo y la distancia
    //rpm fijo talves

	public static void main(String[] args) {
		for (float i = 0; i < 700; i += 10) {
			System.out.println(i);
			System.out.println(getPoint(i));
		}
	}

	public record DataPoint(double distance, double rpm, double angle) {}

	public static DataPoint[] table = {
			new DataPoint(21, 210, 2.1),
			new DataPoint(82, 820, 8.2),
			new DataPoint(112, 1120, 11.2),
			new DataPoint(590, 5900, 59)
	};

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

	public static DataPoint interpolatePoint(DataPoint a, DataPoint b, double desiredDistance) {

		double t = (desiredDistance - a.distance) / (b.distance - a.distance);

		return new DataPoint(
				lerp(a.distance, b.distance, t),
				lerp(a.rpm, b.rpm, t),
				lerp(a.angle, b.angle, t)
		);
	}

	public static double lerp(double a, double b, double t) {
		return a + (b - a) * t;
	}
}
// metodo de interolacion