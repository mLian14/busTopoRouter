package processor;

import grb.GurobiExecutor;
import parser.DocumentParser;
import parser.Document;
import parser.OutputDocument;
import shapes.*;

import java.util.ArrayList;

/**
 * @auther lianmeng
 * @create 26.03.23
 */
public class Processor {


    private GurobiExecutor executor;
    private final DocumentParser parser;
    private final static int M = 999999;
    private OutputDocument output;


    public Processor() {
        this.parser = new DocumentParser();
    }

    public OutputDocument processToOutputPre(String path) {

        parser.parseInputToDocument(path);
        Document input = parser.getParseDoc();
        String[] names = path.split("/");
        input.setName(names[names.length - 1]);
        return processToOutputPre(input.getName(), input.getMaster(), input.getSlaves(), input.getObstacles());
    }

    /**
     * Process all the information from the board.
     * Preparation process.
     *
     * @param caseName  the name of the case
     * @param master    master
     * @param slaves    ArrayList of slaves
     * @param obstacles ArrayList of obstacles
     * @return OutputDocument from preparation
     */
    public OutputDocument processToOutputPre(String caseName, PseudoBase master, ArrayList<PseudoBase> slaves, ArrayList<Obstacle> obstacles) {
        OutputDocument output = new OutputDocument(caseName);
        pseudoBaseVariablesDetermination(master, slaves, obstacles);


        return output;
    }

    /**
     * Determine the overlapped obstacle within the PathArea
     *
     * @param obstacles
     */
    public void detectOverlappedObstacleWithinPathArea(ArrayList<Obstacle> obstacles) {

        for (int i = 0; i < obstacles.size(); ++i) {
            Obstacle cur_o = obstacles.get(i);

            for (int j = i + 1; j < obstacles.size(); ++j) {
                Obstacle other_o = obstacles.get(j);

                //LL and LL
                PseudoBase cur_o_LL = cur_o.getLowerLeft();
                PseudoBase other_o_LL = other_o.getLowerLeft();

                PseudoBase p1 = cur_o_LL;
                PseudoBase p3 = other_o_LL;
                PseudoBase p2 = null, p4 = null;
                int dx = Math.abs(cur_o_LL.getX() - other_o_LL.getX());
                int dy = Math.abs(cur_o_LL.getY() - other_o_LL.getY());
                if (dx == dy) {
                    p2 = cur_o_LL;
                    p4 = other_o_LL;
                }

                else {
                    int plusDY = (int) Math.max(dy * Math.signum(dx - dy), 0);
                    int plusDx = (int) Math.max(dx * Math.signum(dy - dx), 0);
                    //other: lowerRight
                    if (other_o_LL.getX() > cur_o_LL.getX() && other_o_LL.getY() < cur_o_LL.getY()) {
                        p2 = new PseudoBase(other_o_LL.getX() - plusDY, cur_o_LL.getY() - plusDx);
                        p4 = new PseudoBase(cur_o_LL.getX() - plusDY, other_o_LL.getY() - plusDx);
                    }
                    //other: upperRight
                    else if (other_o_LL.getX() > cur_o_LL.getX() && other_o_LL.getY() > cur_o_LL.getY()) {
                        p2 = new PseudoBase(cur_o_LL.getX() + plusDY, other_o_LL.getY() - plusDx);
                        p4 = new PseudoBase(other_o_LL.getX() - plusDY, cur_o_LL.getY() + plusDx);
                    }
                    //other: upperLeft
                    else if (other_o_LL.getX() < cur_o_LL.getX() && other_o_LL.getY() > cur_o_LL.getY()) {
                        p2 = new PseudoBase(cur_o_LL.getX() - plusDY, other_o_LL.getY() - plusDx);
                        p4 = new PseudoBase(other_o_LL.getX() + plusDY, cur_o_LL.getY() + plusDx);
                    }
                    //other: lowerLeft
                    else if (other_o_LL.getX() < cur_o_LL.getX() && other_o_LL.getY() < cur_o_LL.getY()){
                        p2 = new PseudoBase(other_o_LL.getX() + plusDY, cur_o_LL.getY() - plusDx);
                        p4 = new PseudoBase(cur_o_LL.getX() - plusDY, other_o_LL.getY() + plusDx);
                    }
                    else {
                        System.err.println("Another Type of Parallelogram!!");
                    }

                }

                ArrayList<Obstacle> overlappedO = new ArrayList<>();
                for (Obstacle o : obstacles) {
                    //check if one of the edges of obstacles v.s., one of the edges of the parallelogram
                    if (segmentOverlappedObstacle(p1, p2, overlappedO, o)) continue;
                    if (segmentOverlappedObstacle(p2, p3, overlappedO, o)) continue;
                    if (segmentOverlappedObstacle(p3, p4, overlappedO, o)) continue;
                    if (segmentOverlappedObstacle(p4, p1, overlappedO, o)) continue;

                }


            }

        }

    }

    private boolean segmentOverlappedObstacle(PseudoBase p1, PseudoBase p2, ArrayList<Obstacle> overlappedO, Obstacle o) {
        //p1_p2 vs o_LL_LR
        if (doIntersect(p1, o.getLowerLeft(), p2, o.getLowerRight())) {
            overlappedO.add(o);
            return true;
        }
        //p1_p2 vs o_LL_UL
        if (doIntersect(p1, o.getLowerLeft(), p2, o.getUpperLeft())) {
            overlappedO.add(o);
            return true;
        }
        //p1_p2 vs o_UR_UL
        if (doIntersect(p1, o.getUpperRight(), p2, o.getUpperLeft())) {
            overlappedO.add(o);
            return true;
        }
        //p1_p2 vs o_UR_LR
        if (doIntersect(p1, o.getUpperRight(), p2, o.getLowerRight())) {
            overlappedO.add(o);
            return true;
        }
        return false;
    }


    /**
     * Detect the direction and relation between known points, i.e., Master and Slaves, and obstacles.
     *
     * @param master    master
     * @param slaves    ArrayList of slaves
     * @param obstacles ArrayList of obstacles
     */
    public void pseudoBaseVariablesDetermination(PseudoBase master, ArrayList<PseudoBase> slaves, ArrayList<Obstacle> obstacles) {

        for (Obstacle o : obstacles) {

            basicBinaryVariables(master, o);
            for (PseudoBase slave : slaves) {
                basicBinaryVariables(slave, o);
            }


        }


    }

    /**
     * Determine dir_q and rel_q
     *
     * @param base given point
     * @param o    given obstacle
     */
    private void basicBinaryVariables(PseudoBase base, Obstacle o) {

        /*
        Dir_q: L, R, A, B, UL, UR, LR, LL, D,U
         */
        int cnt_base_Dir_q = 10;
        int[] dir_q = new int[cnt_base_Dir_q];
        //L
        if (base.getX() < o.getMinX()) {
            dir_q[0] = 1;
        } else dir_q[0] = 0;
        //R
        if (base.getX() > o.getMaxX()) {
            dir_q[1] = 1;
        } else dir_q[1] = 0;
        //A
        if (base.getX() > o.getMaxY()) {
            dir_q[2] = 1;
        } else dir_q[2] = 0;
        //B
        if (base.getY() < o.getMinY()) {
            dir_q[3] = 1;
        } else dir_q[3] = 0;
        //UL
        if (base.getY() <= base.getX() + o.getMaxY() - o.getMinX()) {
            dir_q[4] = 1;
        } else dir_q[4] = 0;
        //UR
        if (base.getY() <= -base.getX() + o.getMaxY() + o.getMaxX()) {
            dir_q[5] = 1;
        } else dir_q[5] = 0;
        //LR
        if (base.getY() <= base.getX() + o.getMinY() - o.getMaxX()) {
            dir_q[6] = 1;
        } else dir_q[6] = 0;
        //LL
        if (base.getY() <= -base.getX() + o.getMinY() + o.getMinX()) {
            dir_q[7] = 1;
        } else dir_q[7] = 0;
        //D
        if (base.getY() <= (double) (o.getMinY() - o.getMaxY()) / (double) (o.getMaxX() - o.getMinX()) * (double) (base.getX() - o.getMinX()) + o.getMaxY()) {
            dir_q[8] = 1;
        } else dir_q[8] = 0;
        //U
        if (base.getY() <= (double) (o.getMaxY() - o.getMinY()) / (double) (o.getMaxX() - o.getMinX()) * (double) (base.getX() - o.getMinX()) + o.getMinY()) {
            dir_q[9] = 1;
        } else dir_q[9] = 0;
        base.addToPseudo_Dir_qs(o, dir_q);

        /*
        Rel_q:
         */
        int cnt_Rel_q = 8;
        int[] rel_q = new int[cnt_Rel_q];
        //Ld
        if (dir_q[0] == 1 && dir_q[2] + dir_q[3] == 0) {
            rel_q[0] = 1;
        } else rel_q[0] = 0;
        //Rd
        if (dir_q[1] == 1 && dir_q[2] + dir_q[3] == 0) {
            rel_q[1] = 1;
        } else rel_q[1] = 0;
        //Ad
        if (dir_q[2] == 1 && dir_q[0] + dir_q[1] == 0) {
            rel_q[2] = 1;
        } else rel_q[2] = 0;
        //Bd
        if (dir_q[3] == 1 && dir_q[0] + dir_q[1] == 0) {
            rel_q[3] = 1;
        } else rel_q[3] = 0;
        //UpperLeft
        if (dir_q[7] + dir_q[9] == 0 && dir_q[5] == 1) {
            rel_q[4] = 1;
        } else rel_q[4] = 0;
        //UpperRight
        if (dir_q[6] + dir_q[8] == 0 && dir_q[4] == 1) {
            rel_q[5] = 1;
        } else rel_q[5] = 0;
        //LowerLeft
        if (dir_q[4] + dir_q[8] == 2 && dir_q[6] == 0) {
            rel_q[6] = 1;
        } else rel_q[6] = 0;
        //LowerRight
        if (dir_q[5] + dir_q[9] == 2 && dir_q[7] == 0) {
            rel_q[7] = 1;
        } else rel_q[7] = 0;


    }


    public void pathDeterminationAlgorithm(ArrayList<Obstacle> obstacles) {


    }


    public static boolean onSegment(PseudoBase p, PseudoBase q, PseudoBase r) {
        if (q.getX() <= Math.max(p.getX(), r.getX()) && q.getX() >= Math.min(p.getX(), r.getX())
                && q.getY() <= Math.max(p.getY(), r.getY()) && q.getY() >= Math.min(p.getY(), r.getY()))
            return true;
        return false;
    }

    public static int orientation(PseudoBase p, PseudoBase q, PseudoBase r) {
        double val = (q.getY() - p.getY()) * (r.getX() - q.getX()) - (q.getX() - p.getX()) * (r.getY() - q.getY());

        if (val == 0)
            return 0;
        return (val > 0) ? 1 : 2;
    }

    // The main function that returns true if line segment 'p1q1' and 'p2q2' intersect.
    public static boolean doIntersect(PseudoBase p1, PseudoBase q1, PseudoBase p2, PseudoBase q2) {

        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        if (o1 != o2 && o3 != o4)
            return true;

        if (o1 == 0 && onSegment(p1, p2, q1))
            return true;

        if (o2 == 0 && onSegment(p1, q2, q1))
            return true;

        if (o3 == 0 && onSegment(p2, p1, q2))
            return true;

        if (o4 == 0 && onSegment(p2, q1, q2))
            return true;

        return false;
    }

}
