package c2g2.kinematics;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import org.joml.Vector2d;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

/* Class that stores the tree structure of the skeleton. */
public class Skeleton2D {

    private Joint2D root = null;

    public Joint2D getRoot() { return root; }
    public void setRoot(Joint2D r) { root = r; }
    
    // empty constructor
    public Skeleton2D() {}

    // construct skeleton from xml file
    public Skeleton2D(String xmlFile) {
        try {   
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(new File(xmlFile));

            doc.getDocumentElement().normalize();
            System.out.println("Build skeleton: ");
            NodeList nList = doc.getElementsByTagName("root");
            if (nList.getLength() != 1) {
                System.out.println("There can only be ONE ROOT in a skeleton!");
            }
            Element rootElement = (Element) nList.item(0);

            double x = Double.parseDouble(rootElement.getAttribute("x"));
            double y = Double.parseDouble(rootElement.getAttribute("y"));
            System.out.println("root: ("+x+", "+y+")");
            root = new Joint2D(new Vector2d(x, y)); 

            buildSkeletonFromDoc(rootElement, root);
            System.out.println("----------------------------");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // helper to recursively build the skeleton
    private void buildSkeletonFromDoc(Element e, Joint2D pJoint) {
        NodeList nList = e.getChildNodes();

        for (int i = 0; i < nList.getLength(); i++) {
            if (nList.item(i).getNodeType() == Node.ELEMENT_NODE) {
                Element cElement = (Element) nList.item(i);
                if (cElement == null) { continue; }

                double len = Double.parseDouble(cElement.getAttribute("length"));
                double ang = Double.parseDouble(cElement.getAttribute("angle"));
                System.out.println("c: len="+len+"; ang="+ang);
                Joint2D cJoint = new Joint2D(pJoint, len, pJoint.getRotateAngle()+ang);
                pJoint.addChildJoint(cJoint);
                buildSkeletonFromDoc(cElement, cJoint);
            } 
        }
    }

    // Write the current skeleton pose to an xml file. 
    public void printPoseToXML(int n) throws IOException {
        FileWriter fileWriter = new FileWriter("src/resources/frames/pose"+n+".xml");
        PrintWriter printWriter = new PrintWriter(fileWriter);
        Vector2d pPos = root.getPos();
        printWriter.printf("<?xml version=\"1.0\"?>\n<root x=\"%.2f\" y=\"%.2f\">\n",
            pPos.x, pPos.y);
        printJoint(root, printWriter, "  ");
        printWriter.println("</joint>");
        printWriter.close();
    }

    // helper to print the pose. 
    private void printJoint(Joint2D pJoint, PrintWriter pw, String pre) {
        if (!pJoint.isLeaf()) {
            ArrayList<Joint2D> cJoints = pJoint.getChildJoints();
            for (Joint2D cJoint : cJoints){
                pw.printf("%s<joint length=\"%.2f\" angle=\"%.2f\">\n", pre, 
                    cJoint.getLength(), cJoint.getRotateAngle()-pJoint.getRotateAngle());
                if (cJoint.isLeaf()) { continue; }
                printJoint(cJoint, pw, pre+"  ");
                pw.printf("%s</joint>\n", pre);
            }
        }
    }


    // Return the set of angles of the current skeleton.
    public double[] getAngles() {
        ArrayList<Double> angList = new ArrayList<>();
        visitNode(root, angList);
        return angList.stream().mapToDouble(Double::doubleValue).toArray();
    }


    // helper for getting the angles. 
    private void visitNode(Joint2D pJoint, ArrayList<Double> angList) {
        ArrayList<Joint2D> cJoints = pJoint.getChildJoints();
        for (Joint2D cJoint: cJoints) {
            angList.add(cJoint.getRotateAngle()-pJoint.getRotateAngle());
            if (cJoint.isLeaf()) { continue; }
            visitNode(cJoint, angList);
        }
    }

    public ArrayList<Joint2D> getJoints(){
        ArrayList<Joint2D> joints = new ArrayList<>();
        getJointsDFS(getRoot(), joints);
        return joints;

    }

    private void getJointsDFS(Joint2D pJoint, ArrayList<Joint2D> joints){
        ArrayList<Joint2D> cJoints = pJoint.getChildJoints();
        for (Joint2D cJoint: cJoints) {
            joints.add(cJoint);
            if (cJoint.isLeaf()) { continue; }
            getJointsDFS(cJoint, joints);
        }

    }

}
