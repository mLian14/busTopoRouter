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

    public OutputDocument processToOutput(String path){

        parser.parseInputToDocument(path);
        Document input = parser.getParseDoc();
        String[] names = path.split("/");
        input.setName(names[names.length - 1]);
        return processToOutput(input.getName(), input.getMaster(), input.getSlaves(), input.getObstacles(), input.getBusC(), input.getSlaveC());
    }

    public OutputDocument processToOutput(String caseName, PseudoBase master, ArrayList<PseudoBase> slaves, ArrayList<Obstacle> obstacles, double busCoefficient, double branchCoefficient){
        OutputDocument output = new OutputDocument(caseName);

        pseudoBaseVariablesDetermination(master, slaves, obstacles);




        return output;
    }

    public void pseudoBaseVariablesDetermination(PseudoBase master, ArrayList<PseudoBase> slaves, ArrayList<Obstacle> obstacles){

        for (Obstacle o : obstacles){

            basicBinaryVariables(master, o);
            for (PseudoBase slave : slaves){
                basicBinaryVariables(slave, o);
            }


        }



    }

    private void basicBinaryVariables(PseudoBase base, Obstacle o) {

        /*
        Dir_q: L, R, A, B, UL, UR, LR, LL, D,U
         */
        int cnt_base_Dir_q = 10;
        int[] dir_q = new int[cnt_base_Dir_q];
        //L
        if (base.getX() < o.getMinX()){
            dir_q[0] = 1;
        }else dir_q[0] = 0;
        //R
        if (base.getX() > o.getMaxX()){
            dir_q[1] = 1;
        }else dir_q[1] = 0;
        //A
        if (base.getX() > o.getMaxY()){
            dir_q[2] = 1;
        }else dir_q[2] = 0;
        //B
        if (base.getY() < o.getMinY()){
            dir_q[3] = 1;
        }else dir_q[3] = 0;
        //UL
        if (base.getY() <= base.getX() + o.getMaxY() - o.getMinX()){
            dir_q[4] = 1;
        }else dir_q[4] = 0;
        //UR
        if (base.getY() <= -base.getX() + o.getMaxY() + o.getMaxX()){
            dir_q[5] = 1;
        }else dir_q[5] = 0;
        //LR
        if (base.getY() <= base.getX() + o.getMinY() - o.getMaxX()){
            dir_q[6] = 1;
        }else dir_q[6] = 0;
        //LL
        if (base.getY() <= -base.getX() + o.getMinY() + o.getMinX()){
            dir_q[7] = 1;
        }else dir_q[7] = 0;
        //D
        if (base.getY() <= (double)(o.getMinY() - o.getMaxY())/(double)(o.getMaxX() - o.getMinX()) * (double) (base.getX() - o.getMinX()) + o.getMaxY()){
            dir_q[8] = 1;
        }else dir_q[8] = 0;
        //U
        if (base.getY() <= (double)(o.getMaxY() - o.getMinY())/(double) (o.getMaxX() - o.getMinX()) * (double) (base.getX() - o.getMinX()) + o.getMinY()){
            dir_q[9] = 1;
        }else dir_q[9] = 0;
        base.addToPseudo_Dir_qs(o, dir_q);

        /*
        Rel_q:
         */
        int cnt_Rel_q = 8;
        int[] rel_q = new int[cnt_Rel_q];
        //Ld
        if (dir_q[0] == 1 && dir_q[2] + dir_q[3] == 0){
            rel_q[0] = 1;
        }else rel_q[0] = 0;
        //Rd
        if (dir_q[1] == 1 && dir_q[2] + dir_q[3] == 0){
            rel_q[1] = 1;
        }else rel_q[1] = 0;
        //Ad
        if (dir_q[2] == 1 && dir_q[0] + dir_q[1] == 0){
            rel_q[2] = 1;
        }else rel_q[2] = 0;
        //Bd
        if (dir_q[3] == 1 && dir_q[0] + dir_q[1] == 0){
            rel_q[3] = 1;
        }else rel_q[3] = 0;
        //UpperLeft
        if (dir_q[7] + dir_q[9] ==0 && dir_q[5] == 1){
            rel_q[4] = 1;
        }else rel_q[4] = 0;
        //UpperRight
        if (dir_q[6] + dir_q[8] == 0 && dir_q[4] == 1){
            rel_q[5] = 1;
        }else rel_q[5] = 0;
        //LowerLeft
        if (dir_q[4] + dir_q[8] == 2 && dir_q[6] == 0){
            rel_q[6] = 1;
        }else rel_q[6] = 0;
        //LowerRight
        if (dir_q[5] + dir_q[9] == 2 && dir_q[7] == 0){
            rel_q[7] = 1;
        }else rel_q[7] = 0;











    }


    public void pathDeterminationAlgorithm(ArrayList<Obstacle> obstacles){



    }

}
