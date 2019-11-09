package c2g2.kinematics;

import org.joml.Matrix3d;
import org.joml.Vector2d;
import org.joml.Vector3d;
import org.lwjgl.system.CallbackI;

import java.util.ArrayList;

/*
 * This is the class to implement your inverse kinematics algorithm.
 * 
 * TODO:
 * Please include your data structure and methods for the inverse kinematics here.
 */
public class InverseKinematics {


    private Skeleton2D skeleton;
    // Feel free to add more variables as needed.
    private double epsilon = 1e-6;
    private ArrayList<Double> lengths = new ArrayList<>();


    public InverseKinematics(Skeleton2D ske) {
        if (ske == null) throw new NullPointerException("The provided skeleton is NULL!");
        skeleton = ske;
    }

   //https://medium.com/unity3danimation/overview-of-jacobian-ik-a33939639ab2
    // Add methods for updating the skeleton.


    public void dragJointTo(Joint2D joint, Vector2d dest){
        ArrayList<Double> thetas = new ArrayList<>();
        initParams(joint, thetas);

        Vector2d v = new Vector2d(); //will be distance between curr and dest pos
        Vector2d curr = joint.getPos();
        Vector2d prev = new Vector2d();
        Vector2d prePrev = new Vector2d();

        //iteration
        ArrayList<Vector2d> J;
        ArrayList<Double> thetasBar = new ArrayList<>(thetas);


        while (curr.distance(dest)>0.01 && !prev.equals(curr) && !prePrev.equals(curr)){
            System.out.println(curr.distance(dest));
            J = getJ(thetas);
            v.set(dest);
            v.sub(curr);

            double [] deltaTheta = getDeltaTheta(J, v, thetasBar);
            if (deltaTheta == null){
                break;
            }


            for(int i =0; i<thetas.size(); i++){
                double currTheta = thetas.get(i);
                currTheta = currTheta + deltaTheta[i];
                double thetaBar = currTheta - thetas.get(i);
                thetasBar.set(i,thetaBar);
                thetas.set(i, currTheta);
            }
            prePrev = prev;
            prev = curr;
            curr = newPos(thetas);
        }

        update(joint, thetas, thetas.size());

    }

    private void initParams(Joint2D joint, ArrayList<Double> thetas){
        //load to front so list starts with right after root

        if (joint.isRoot()){
            return;
        }
        thetas.add(0,joint.getRotateAngle());
        lengths.add(0,joint.getLength());
        initParams(joint.getParentJoint(), thetas);
        return;
    }

    private Vector2d newPos(ArrayList<Double> thetas){


        Vector2d pos = new Vector2d(skeleton.getRoot().getPos());

        for(int i = 0; i<thetas.size(); i++){
            double angle = thetas.get(i);
            double length = lengths.get(i);
            double cos = Math.cos(angle);
            double sin = Math.sin(angle);
            Matrix3d T = new Matrix3d(cos, sin, 0,
                    -sin, cos, 0,
                    pos.x, pos.y, 1);
            Vector3d res = (new Vector3d(length, 0, 1)).mul(T);
            pos.set(res.x,res.y);
        }
        return pos;
    }

    private ArrayList<Vector2d> getJ(ArrayList<Double> thetas){
        ArrayList<Vector2d> jacobian = new ArrayList<>();
        ArrayList<Double> thetaKs=new ArrayList<>(thetas);

        Vector2d v1;
        Vector2d v2;

        double deltaTheta = 0.01;

        for(int i = 0; i<thetas.size(); i++){

            double currTheta = thetas.get(i);

            thetaKs.set(i, currTheta + deltaTheta);
            v1 = newPos(thetaKs);

            thetaKs.set(i, currTheta - deltaTheta);
            v2 = newPos(thetaKs);

            jacobian.add(new Vector2d((v1.x()-v2.x())/(2*deltaTheta),(v1.y()-v2.y())/(2*deltaTheta)));

            thetaKs.set(i,thetas.get(i));
        }
        return jacobian;
    }

    private double[] getDeltaTheta(ArrayList<Vector2d> jacobian, Vector2d v, ArrayList<Double> thetaBar){
        int n = jacobian.size();
        double[][] Jlagr = new double[n+2][n+2];
        double[] B = new double [n+2];

        for (int i = 0; i<n; i++){
            B[i] = 2 * thetaBar.get(i);
        }
        B[n] = v.x;
        B[n+1] = v.y;

        for (int i=0;i<n+2;i++){
            for(int j=0;j<n+2;j++){
                if(i<n && j<n){//in 2I
                    if(i == j) Jlagr[i][j] = 2;
                    else Jlagr[i][j] = 0;
                }
                else if(i>=n && j<n){
                    if(i == n)Jlagr[i][j] = jacobian.get(j).x;
                    if(i==n+1)Jlagr[i][j] = jacobian.get(j).y;
                }
                else if(j>=n && i<n){
                    if(j == n)Jlagr[i][j] = jacobian.get(i).x;
                    if(j==n+1)Jlagr[i][j] = jacobian.get(i).y;
                }
                else Jlagr[i][j] = 0;

            }
        }

        double[][] invJlagr = invertMat(Jlagr);
        double[] A = B.clone();
        multMatVec(invJlagr, B, A);
        return A;
    }

    private void multMatVec(double[][] M, double[] v, double[] dest){
        int n = M.length;

        for (int i=0; i< n ; i++){
            dest[i]=0;
            for (int j=0; j<n; j++){
                dest[i] += M[i][j]*v[j];
            }
        }
    }
    private void update(Joint2D joint, ArrayList<Double> thetas, int index){
        if (!joint.isRoot()){
            joint.setRotateAngle(thetas.get(index-1));
            joint.setPos(newPos(thetas));
            thetas.remove(index-1);
            update(joint.getParentJoint(),thetas, index-1);
        }
        return;
    }

    private double[][] invertMat(double[][] M){
        int n = M.length;
        int m = M[0].length;
        double det = det(M);
        if (Math.abs(det)<epsilon) return null;
        double sI=1;
        double[][] inv = new double[n][n];

        for (int i= 0; i< n; i++){
            double sj=1;
            for (int j = 0 ; j<n; j++){

                double [][] Mij = new double[n-1][m-1];
                for (int p=1; p<n;p++){
                    for (int q=1;q<m;q++){
                        int x = p;
                        int y = q;
                        if(p<=i) x--;
                        if(y<=j) y--;
                        Mij[p-1][q-1] = M[x][y];
                    }
                }

                inv[i][j] = sI * sj * det(Mij)/det;
                sj *= -1;
            }
            sI *= -1;
        }
        return inv;
    }

    private double det(double[][] M){
        int n = M.length;
        int m = M[0].length;
        if (n==1) return M[0][0];
        else {
            double det = 0;
            double s = 1;
            for (int i = 0; i < n; i++) {

                double [][] M0i = new double[n-1][m-1];
                for (int p=1; p<n;p++){
                    for (int q=1;q<m;q++){
                        int x = p;
                        int y = q;
                        if(p<=0) x--;
                        if(y<=i) y--;
                        M0i[p-1][q-1] = M[x][y];
                    }
                }

                det = det + (s * M[0][i] * det(M0i));
                s *= -1;
            }
            return det;
        }
    }

}
