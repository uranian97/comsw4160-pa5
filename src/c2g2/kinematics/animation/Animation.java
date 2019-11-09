package c2g2.kinematics.animation;

import c2g2.kinematics.*;
import java.io.File;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import org.joml.Vector2d;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

/* Class for animating the skeleton with kinematics. */
public class Animation {

	private final String LINEAR = "linear";
	private final String CUBICBEZIER = "cubicbezier";
	private final String DIR = "src/resources/frames/";
	private final int MILLI = 1000; // 1 second = 1000 milliseconds

	public boolean isAnimating;

	private long period; // # of milliseconds per frame
    private int fps; // frame rate
    public int getFPS() { return fps; }

	private int N; // number of frames total
    private double t; // interpolation parameter [0, 1]
    private Timer timer;

	private Interpolator itp; // the interpolator
    private ForwardKinematics fk; // display the interpolated pose

    // constructor;
    // takes in forward kinematics and the filename of the configuration file. 
	public Animation(ForwardKinematics fk, String configFile) {
		this.fk = fk;
		isAnimating = false;
		timer = new Timer();
        t = 0.0;

        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document config = dBuilder.parse(new File(configFile));
            config.getDocumentElement().normalize();

            NodeList nList = config.getElementsByTagName("animation");
            if (nList.getLength() != 1) {
                System.out.println("There can only be ONE ANIMATION in a config file!");
            }
            Element anim = (Element) nList.item(0);
            fps = Integer.parseInt(anim.getAttribute("fps"));
            period = MILLI / fps;

            ArrayList<double[]> frames = new ArrayList<>();
            ArrayList<Integer> times = new ArrayList<>();

            nList = config.getElementsByTagName("frame");
            for (int i = 0; i < nList.getLength(); i++) {
                if (nList.item(i).getNodeType() == Node.ELEMENT_NODE) {
                    Element frameElem = (Element) nList.item(i);
                    if (frameElem == null) { continue; }
                    frames.add(parseAngles(frameElem.getTextContent().trim()));
                    times.add(Integer.parseInt(frameElem.getAttribute("time")));
                }
            }

            // Assume the time stamps in the config file are sorted. 
            N = (times.get(times.size()-1) - times.get(0)) * fps;

            String itpType = anim.getAttribute("interp");
            if (itpType.equals(LINEAR)) {
            	itp = new Linear(frames, times, N);
            } else if (itpType.equals(CUBICBEZIER)) {
            	itp = new CubicBezier(frames, times, fps);
            // you can add more interpolators here. 
            } else {
            	throw new IllegalArgumentException("No such interpolator!");
            }
            
        } catch (Exception e) {
            e.printStackTrace();
        }
	}

    // helper method to read angles from the config file.
    private double[] parseAngles(String arr) {
        arr = arr.substring(1, arr.length()-1);
        String[] angles = arr.split(", ");
        ArrayList<Double> ang = new ArrayList<>();
        for (int i = 0; i < angles.length; i++) {
            ang.add(Double.parseDouble(angles[i]));
        }
        return ang.stream().mapToDouble(Double::doubleValue).toArray();
    }

    // play the animation. 
	public void playAnimation() {
		isAnimating = true;

        timer.schedule(new TimerTask() {
            public void run() {
                if ((t + 1.0/N) >= 1) {
                    // Stop timer and cancel tasks
                    t = 0.0;
                    isAnimating = false;
                    timer.cancel();
                    timer.purge();
                    System.out.println("Timer terminated.");
					timer = new Timer();
                    return;
                }

                t += 1.0 / N;
                fk.updateState(itp.interp(t));
            }
        }, 0, period);
	}
}