package c2g2.kinematics.engine;

public class Main {
    public static void main(String[] args) {
        try {
            Scene scene = new Scene();
            // for example: check out the XML file src/resources/models/test.xml
            String inputXMLFile = args[0];
            scene.loadfromXML(inputXMLFile);
            Renderer r = new Renderer(scene);
            r.run();
        } catch (NullPointerException e) {
             e.printStackTrace(); // uncomment this line if window closes unexpectedly
            System.out.println("Window closed; terminate. ");
            System.exit(0);
        }
    }
}
