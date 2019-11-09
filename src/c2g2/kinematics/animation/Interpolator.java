package c2g2.kinematics.animation;

import java.util.ArrayList;

public abstract class Interpolator {
	protected ArrayList<double[]> kfAngles;
	protected int[] kfTimestamps;
	protected int fps;

	public Interpolator(ArrayList<double[]> frames, ArrayList<Integer> times, int fps) {
		kfAngles = frames;
		kfTimestamps = times.stream().mapToInt(Integer::intValue).toArray();
		this.fps = fps;
	}

	public abstract double[] interp(double t);
}