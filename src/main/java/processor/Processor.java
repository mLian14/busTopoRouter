package processor;

import grb.*;
import gurobi.GRB;
import gurobi.GRBException;
import parser.DocumentParser;
import parser.Document;
import parser.OutputDocument;
import shapeVar.VirtualPointVar;
import shapes.*;

import java.util.*;

/**
 * @auther lianmeng
 * @create 26.03.23
 */
public class Processor {


    private GurobiExecutor executor;
    private final DocumentParser parser;
    private final static int M = 999999;
    private OutputDocument output;

    enum SortType {
        LeftToRight,
        RightToLeft,
        TopToBottom,
        BottomToTop
    }

    enum OppositeType {
        topLtoBottomR,
        bottomLtoTopR,
        topRtoBottomL,
        bottomRtoTopL,
        NoOppositeRelation
    }


    public Processor() {
        this.parser = new DocumentParser();
    }

    public OutputDocument processToOutputPre(String path) throws GRBException {

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
    public OutputDocument processToOutputPre(String caseName, PseudoBase master, ArrayList<PseudoBase> slaves, ArrayList<Obstacle> obstacles) throws GRBException {
        OutputDocument output = new OutputDocument(caseName);
        /*
        Determine the pseudo Variables of Master and slaves
         */
        pseudoBaseVariablesDetermination(master, slaves, obstacles);

        executor = new GurobiExecutor("LinearBusRouting_KO");
        executor.setMIPGap(0);
        executor.setMIPGapAbs(0);

        //build Gurobi Variables
        GurobiVariable busLength, branchLength;
        busLength = new GurobiVariable(GRB.CONTINUOUS, 0, M, "busLength");
        executor.addVariable(busLength);
        branchLength = new GurobiVariable(GRB.CONTINUOUS, 0,M, "branchLength");
        executor.addVariable(branchLength);

        ArrayList<VirtualPointVar> vps = new ArrayList<>();
        buildVars(obstacles, vps, slaves, master);
        executor.updateModelWithVars();
        System.out.println("#Variables = " + executor.getVariables().size());

        //build Constraints
        buildCons(obstacles, vps, slaves, master, busLength, branchLength);
        executor.updateModelWithCons();
        System.out.println("#Constraints = " + executor.getConstraints().size());

        //Objective Function
        GurobiObjConstraint objCons = new GurobiObjConstraint();
        objCons.addToLHS(busLength, 1.0);
        objCons.addToLHS(branchLength, 1.0);
        objCons.setGoal(GRB.MINIMIZE);
        executor.setObjConstraint(objCons);

        executor.startOptimization();
        this.executor.getModel().write("debug.lp");
        if (executor.getModel().get(GRB.IntAttr.Status) == GRB.INFEASIBLE) {
            executor.whenInfeasibleWriteOutputTo("inf.ilp");
            System.out.println("HH:IIS");
            //break;
        }
//        System.out.println("C975 = " + executor.getConstraints().get(149));
//        System.out.println("C975 = " + executor.getConstraints().get(214));
//        System.out.println("C975 = " + executor.getConstraints().get(215));
//        System.out.println("C975 = " + executor.getConstraints().get(216));
//        System.out.println("C975 = " + executor.getConstraints().get(222));
//        System.out.println("C975 = " + executor.getConstraints().get(288));
//        System.out.println("C975 = " + executor.getConstraints().get(289));
//        System.out.println("C975 = " + executor.getConstraints().get(304));
//        System.out.println("C975 = " + executor.getConstraints().get(359));




        /*
        Retrieve
         */
        GurobiVariable q;
        for (int i = 0; i < vps.size(); ++i){
            VirtualPointVar vp = vps.get(i);
            System.out.println("vp" + i + "=(" + vp.x.getIntResult() + ", " + vp.y.getIntResult() + ").");
            for (Obstacle o : obstacles){
                System.out.print(o.getName() + " " + o.getMinX() + " " + o.getMaxX() + " " + o.getMinY() + " " + o.getMaxY() + "|| ");
                System.out.println("nonL:" + convertGrbIntArrayToString(vp.non_qs.get(o)));
            }


            System.out.println("vp" + i + "->Slave Relevant VVVVVVVVVVVVVVVVVVVVVVVVVV");
            for (PseudoBase sv : slaves){
                if (vp.vsCnn_q.get(sv).getIntResult() == 1){
                    System.out.println("vp" + i + "=(" + vp.x.getIntResult() + ", " + vp.y.getIntResult() + ")->" + sv.getName() + ":(" + sv.getX() + ", " + sv.getY() + ")");
                    System.out.println("d_v->CorrS= " + vp.dist_cqs[1].getContResult());
                    System.out.println("aux_vsDist_cqs=" + convertGrbContArrayToString(vp.aux_vsDist_cqs.get(sv)));
                }




            }
            System.out.println("vp" + i + "->Slave Relevant AAAAAAAAAAAAAAAAAAAAAAAAAA");

        }






        /*
        Debug
         */
        /*for (Obstacle o : obstacles) {
            System.out.println(o);
        }*/


        return output;
    }

    public void buildCons(ArrayList<Obstacle> obstacles, ArrayList<VirtualPointVar> virtualPointVars, ArrayList<PseudoBase> slaves, PseudoBase master, GurobiVariable busLength, GurobiVariable branchLength) throws GRBException {
        double eps = 1;

        GurobiConstraint c;
        GurobiQuadConstraint qc;

        VirtualPointVar vpN;



        //busLength
        GurobiConstraint c_busLength = new GurobiConstraint();
        c_busLength.addToLHS(busLength, 1.0);
        c_busLength.setSense('=');
        executor.addConstraint(c_busLength);
        //branchLength
        GurobiConstraint c_branchLength = new GurobiConstraint();
        c_branchLength.addToLHS(branchLength, 1.0);
        c_branchLength.setSense('=');
        executor.addConstraint(c_branchLength);

        //sv.2

        for (PseudoBase sv : slaves){
            c = new GurobiConstraint();
            for (VirtualPointVar vp : virtualPointVars){
                c.addToLHS(vp.vsCnn_q.get(sv), 1.0);
            }
            c.setSense('=');
            c.setRHSConstant(1.0);
            executor.addConstraint(c);
        }


        for (VirtualPointVar vp : virtualPointVars) {

            //branchLength
            c_branchLength.addToRHS(vp.dist_cqs[1], 1.0);


            //cnn.4
            GurobiQuadConstraint c_vpOut = new GurobiQuadConstraint();
            c_vpOut.setName("cnn.4");
            c_vpOut.setSense('=');
            c_vpOut.addToRHS(vp.detour_qs[4], 1.0);
            executor.addConstraint(c_vpOut);
            //pl
            GurobiConstraint c_dij_detour = new GurobiConstraint();
            c_dij_detour.setName("pl");
            c_dij_detour.addToLHS(vp.dist_cqs[0], 1.0);
            c_dij_detour.setSense('>');
            c_dij_detour.addToRHS(vp.dInOut_cqs[0], 1.0);
            c_dij_detour.addToRHS(vp.dInOut_cqs[1], 1.0);
            executor.addConstraint(c_dij_detour);

            /*
            Obstacle relative
             */
            for (Obstacle o : obstacles) {
                //nonl
                c = new GurobiConstraint();
                c.setName("nonl");
                c.addToLHS(vp.non_qs.get(o)[0], 1.0);
                c.addToLHS(vp.non_qs.get(o)[1], 1.0);
                c.addToLHS(vp.non_qs.get(o)[2], 1.0);
                c.addToLHS(vp.non_qs.get(o)[3], 1.0);
                c.setSense('=');
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                //nonl.l
                c = new GurobiConstraint();
                c.setName("nonl.l");
                c.addToLHS(vp.x, 1.0);
                c.setSense('<');
                c.addToRHS(vp.non_qs.get(o)[0], -M);
                c.setRHSConstant(o.getMinX() - eps + M);
                executor.addConstraint(c);
                //nonl.r
                c = new GurobiConstraint();
                c.setName("nonl.r");
                c.addToLHS(vp.x, 1.0);
                c.setSense('>');
                c.addToRHS(vp.non_qs.get(o)[1], M);
                c.setRHSConstant(o.getMaxX() + eps - M);
                executor.addConstraint(c);
                //nonl.t
                c = new GurobiConstraint();
                c.setName("nonl.t");
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.non_qs.get(o)[2], M);
                c.setRHSConstant(o.getMaxY() + eps - M);
                executor.addConstraint(c);
                //nonl.b
                c = new GurobiConstraint();
                c.setName("nonl.b");
                c.addToLHS(vp.y, 1.0);
                c.setSense('<');
                c.addToRHS(vp.non_qs.get(o)[3], -M);
                c.setRHSConstant(o.getMinY() - eps + M);
                executor.addConstraint(c);


                //diagonal Sets
                //rel.ul
                c = new GurobiConstraint();
                c.setName("rel.ul");
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, 1.0);
                c.addToRHS(vp.rel_qs.get(o)[0], -M);
                c.setRHSConstant(o.getMaxY() - o.getMinX() + eps);
                executor.addConstraint(c);
                //rel.ur
                c = new GurobiConstraint();
                c.setName("rel.ur");
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, -1.0);
                c.addToRHS(vp.rel_qs.get(o)[1], -M);
                c.setRHSConstant(o.getMaxY() + o.getMaxX() + eps);
                executor.addConstraint(c);
                //rel.lr
                c = new GurobiConstraint();
                c.setName("rel.lr");
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, 1.0);
                c.addToRHS(vp.rel_qs.get(o)[2], -M);
                c.setRHSConstant(o.getMinY() - o.getMaxX() + eps);
                executor.addConstraint(c);
                //rel.ll
                c = new GurobiConstraint();
                c.setName("rel.ll");
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, -1.0);
                c.addToRHS(vp.rel_qs.get(o)[3], -M);
                c.setRHSConstant(o.getMinY() + o.getMinX() + eps);
                executor.addConstraint(c);
                //rel.d
                c = new GurobiConstraint();
                c.setName("rel.d");
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, -o.getGradient());
                c.addToRHS(vp.rel_qs.get(o)[4], -M);
                c.setRHSConstant((-o.getGradient()) * o.getMinX() + o.getMaxY() + eps);
                executor.addConstraint(c);
                //rel.u
                c = new GurobiConstraint();
                c.setName("rel.u");
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, o.getGradient());
                c.addToRHS(vp.rel_qs.get(o)[5], -M);
                c.setRHSConstant(o.getGradient() * o.getMinX() + o.getMinY() + eps);
                executor.addConstraint(c);

                //tL
                c = new GurobiConstraint();
                c.setName("tL.1");
                c.addToLHS(vp.diagonal_qs.get(o)[0], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("tL.2");
                c.addToLHS(vp.diagonal_qs.get(o)[0], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[3], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("tL.3");
                c.addToLHS(vp.diagonal_qs.get(o)[0], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[5], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("tL.4");
                c.addToLHS(vp.diagonal_qs.get(o)[0], 1.0);
                c.setSense('>');
                c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[3], -1.0);
                c.addToRHS(vp.rel_qs.get(o)[5], -1.0);
                executor.addConstraint(c);

                //tR
                c = new GurobiConstraint();
                c.setName("tR.1");
                c.addToLHS(vp.diagonal_qs.get(o)[1], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("tR.2");
                c.addToLHS(vp.diagonal_qs.get(o)[1], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[2], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("tR.3");
                c.addToLHS(vp.diagonal_qs.get(o)[1], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[4], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("tR.4");
                c.addToLHS(vp.diagonal_qs.get(o)[1], 1.0);
                c.setSense('>');
                c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[2], -1.0);
                c.addToRHS(vp.rel_qs.get(o)[4], -1.0);
                executor.addConstraint(c);

                //bL
                c = new GurobiConstraint();
                c.setName("bL.1");
                c.addToLHS(vp.diagonal_qs.get(o)[2], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("bL.2");
                c.addToLHS(vp.diagonal_qs.get(o)[2], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[4], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("bL.3");
                c.addToLHS(vp.diagonal_qs.get(o)[2], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[2], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("bL.4");
                c.addToLHS(vp.diagonal_qs.get(o)[2], 1.0);
                c.setSense('>');
                c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[4], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[2], -1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);

                //bR
                c = new GurobiConstraint();
                c.setName("bR.1");
                c.addToLHS(vp.diagonal_qs.get(o)[3], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("bR.2");
                c.addToLHS(vp.diagonal_qs.get(o)[3], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[5], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("bR.3");
                c.addToLHS(vp.diagonal_qs.get(o)[3], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[3], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("bR.4");
                c.addToLHS(vp.diagonal_qs.get(o)[3], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[5], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[3], -1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);

                /*
                Connection with next virtualPoint
                 */

                if (virtualPointVars.indexOf(vp) < virtualPointVars.size() - 1) {
                    vpN = virtualPointVars.get(virtualPointVars.indexOf(vp) + 1);
                    GurobiConstraint c_relOb;
                    //(ul->lr.1)
                    c_relOb = new GurobiConstraint();
                    c_relOb.setName("ul->lr.1");
                    c_relOb.addToLHS(vp.relObstacles_qs.get(o)[0], M);
                    c_relOb.setSense('>');
                    for (Obstacle on : o.getbRObstacles()) {
                        c_relOb.addToRHS(vpN.diagonal_qs.get(on)[3], 1.0);
                        //(ul->lr.2)
                        c = new GurobiConstraint();
                        c.setName("ul->lr.2");
                        c.addToLHS(vp.relObstacles_qs.get(on)[0], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.diagonal_qs.get(on)[3], 1.0);
                        c.addToRHS(vp.relObstacles_qs.get(o)[0], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);
                    }
                    c_relOb.addToRHS(vp.diagonal_qs.get(o)[0], M);
                    c_relOb.setRHSConstant(-M);
                    executor.addConstraint(c_relOb);
                    //(ul->lr.3)
                    c = new GurobiConstraint();
                    c.setName("ul->lr.3");
                    c.addToLHS(vp.detour_qs[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.diagonal_qs.get(o)[0], 1.0);
                    executor.addConstraint(c);


                    //(lr->ul.1)
                    c_relOb = new GurobiConstraint();
                    c_relOb.setName("lr->ul.1");
                    c_relOb.addToLHS(vp.relObstacles_qs.get(o)[1], M);
                    c_relOb.setSense('>');
                    for (Obstacle on : o.gettLObstacles()) {
                        c_relOb.addToRHS(vpN.diagonal_qs.get(on)[0], 1.0);
                        //(lr->ul.2)
                        c = new GurobiConstraint();
                        c.setName("lr->ul.2");
                        c.addToLHS(vp.relObstacles_qs.get(on)[1], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.diagonal_qs.get(on)[0], 1.0);
                        c.addToRHS(vp.relObstacles_qs.get(o)[1], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);
                    }
                    c_relOb.addToRHS(vp.diagonal_qs.get(o)[3], M);
                    c_relOb.setRHSConstant(-M);
                    executor.addConstraint(c_relOb);
                    //(lr->ul.3)
                    c = new GurobiConstraint();
                    c.setName("lr->ul.3");
                    c.addToLHS(vp.detour_qs[1], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.relObstacles_qs.get(o)[1], 1.0);
                    executor.addConstraint(c);

                    //(ur->ll.1)
                    c_relOb = new GurobiConstraint();
                    c_relOb.setName("ur->ll.1");
                    c_relOb.addToLHS(vp.relObstacles_qs.get(o)[2], M);
                    c_relOb.setSense('>');
                    for (Obstacle on : o.getbLObstacles()) {
                        c_relOb.addToRHS(vpN.diagonal_qs.get(on)[2], 1.0);
                        //(ur->ll.2)
                        c = new GurobiConstraint();
                        c.setName("ur->ll.2");
                        c.addToLHS(vp.relObstacles_qs.get(on)[2], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.diagonal_qs.get(on)[2], 1.0);
                        c.addToRHS(vp.relObstacles_qs.get(o)[2], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);
                    }
                    c_relOb.addToRHS(vp.diagonal_qs.get(o)[1], M);
                    c_relOb.setRHSConstant(-M);
                    executor.addConstraint(c_relOb);
                    //(ur->ll.3)
                    c = new GurobiConstraint();
                    c.setName("ur->ll.3");
                    c.addToLHS(vp.detour_qs[2], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.relObstacles_qs.get(o)[2], 1.0);
                    executor.addConstraint(c);

                    //(ll->ur.1)
                    c_relOb = new GurobiConstraint();
                    c_relOb.setName("ll->ur.1");
                    c_relOb.addToLHS(vp.relObstacles_qs.get(o)[3], M);
                    c_relOb.setSense('>');
                    for (Obstacle on : o.gettRObstacles()) {
                        c_relOb.addToRHS(vpN.diagonal_qs.get(on)[1], 1.0);
                        //(ll->ur.2)
                        c = new GurobiConstraint();
                        c.setName("ll->ur.2");
                        c.addToLHS(vp.relObstacles_qs.get(on)[3], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.diagonal_qs.get(on)[1], 1.0);
                        c.addToRHS(vp.relObstacles_qs.get(o)[3], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);
                    }
                    c_relOb.addToRHS(vp.diagonal_qs.get(o)[2], M);
                    c_relOb.setRHSConstant(-M);
                    executor.addConstraint(c_relOb);
                    //(ll->ur.3)
                    c = new GurobiConstraint();
                    c.setName("ll->ur.3");
                    c.addToLHS(vp.detour_qs[3], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.relObstacles_qs.get(o)[3], 1.0);
                    executor.addConstraint(c);

                    //aux.1
                    c = new GurobiConstraint();
                    c.setName("aux.1");
                    c.addToLHS(vp.relObstacles_qs.get(o)[4], 4.0);
                    c.setSense('>');
                    c.addToRHS(vp.relObstacles_qs.get(o)[0], 1.0);
                    c.addToRHS(vp.relObstacles_qs.get(o)[1], 1.0);
                    c.addToRHS(vp.relObstacles_qs.get(o)[2], 1.0);
                    c.addToRHS(vp.relObstacles_qs.get(o)[3], 1.0);
                    executor.addConstraint(c);
                    //aux.2
                    c = new GurobiConstraint();
                    c.setName("aux.2");
                    c.addToLHS(vp.detour_qs[4], 4.0);
                    c.setSense('>');
                    c.addToRHS(vp.detour_qs[0], 1.0);
                    c.addToRHS(vp.detour_qs[1], 1.0);
                    c.addToRHS(vp.detour_qs[2], 1.0);
                    c.addToRHS(vp.detour_qs[3], 1.0);
                    executor.addConstraint(c);


                    /*
                    Connection Rules:
                     */
                    //cornerCnn
                    c = new GurobiConstraint();
                    c.setName("cornerCnn.1");
                    c.addToLHS(vp.corner_qs.get(o)[0], 1.0);
                    c.addToLHS(vp.corner_qs.get(o)[1], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.relObstacles_qs.get(o)[0], 1.0);
                    c.addToRHS(vp.relObstacles_qs.get(o)[1], 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("cornerCnn.2");
                    c.addToLHS(vp.corner_qs.get(o)[2], 1.0);
                    c.addToLHS(vp.corner_qs.get(o)[3], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.relObstacles_qs.get(o)[2], 1.0);
                    c.addToRHS(vp.relObstacles_qs.get(o)[3], 1.0);
                    executor.addConstraint(c);


                    //cnn.3
                    GurobiQuadConstraint c_outCnn = new GurobiQuadConstraint();
                    c_outCnn.setName("cnn.3");
                    c_outCnn.addToLHS(vp.relObstacles_qs.get(o)[4], 1.0);
                    c_outCnn.setSense('=');
                    c.addToRHS(vp.inOutCnn_qs.get(o)[1], 1.0);
                    executor.addConstraint(c_outCnn);
                    for (Obstacle on : obstacles) {
                        if (o.getName().equals(on.getName())) {
                            //cnn.1
                            c = new GurobiConstraint();
                            c.setName("cnn.1");
                            c.addToLHS(vp.omOnCnn_q.get(o).get(on), 1.0);
                            c.setSense('=');
                            c.setRHSConstant(0.0);
                            executor.addConstraint(c);

                        } else {
                            //cnn.2
                            c = new GurobiConstraint();
                            c.setName("cnn.2");
                            c.addToLHS(vp.omOnCnn_q.get(o).get(on), 1.0);
                            c.addToLHS(vp.omOnCnn_q.get(on).get(o), 1.0);
                            c.setSense('<');
                            c.setRHSConstant(1.0);
                            executor.addConstraint(c);

                            //cnn.3
                            c_outCnn.addToRHS(vp.omOnCnn_q.get(o).get(on), vp.relObstacles_qs.get(on)[4], 1.0);

                            //pl
                            c_dij_detour.addToRHS(vp.dOmOn_cq.get(o).get(on), 1.0);


                        }
                    }

                    //cnn.4 LHS
                    c_vpOut.addToLHS(vp.inOutCnn_qs.get(o)[0], vp.relObstacles_qs.get(o)[4], 1.0);

                    //cor.x
                    c = new GurobiConstraint();
                    c.setName("cor.x");
                    c.addToLHS(vp.oCoordinate_iqs.get(o)[0], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.corner_qs.get(o)[0], o.getMinX());
                    c.addToRHS(vp.corner_qs.get(o)[2], o.getMinX());
                    c.addToRHS(vp.corner_qs.get(o)[1], o.getMaxX());
                    c.addToRHS(vp.corner_qs.get(o)[3], o.getMaxX());
                    executor.addConstraint(c);
                    //cor.y
                    c = new GurobiConstraint();
                    c.setName("cor.y");
                    c.addToLHS(vp.oCoordinate_iqs.get(o)[1], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.corner_qs.get(o)[0], o.getMinY());
                    c.addToRHS(vp.corner_qs.get(o)[3], o.getMinY());
                    c.addToRHS(vp.corner_qs.get(o)[1], o.getMaxY());
                    c.addToRHS(vp.corner_qs.get(o)[2], o.getMaxY());
                    executor.addConstraint(c);

                    //d_ij:-> (pl.1)
                    c = new GurobiConstraint();
                    c.setName("d_ij:-> (pl.1)");
                    c.addToLHS(vp.dInOut_cqs[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dOut_cqs.get(o)[2], Math.sqrt(2));
                    c.addToRHS(vp.aux_dOut_cqs.get(o)[3], 1.0);
                    c.addToRHS(vp.inOutCnn_qs.get(o)[0], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    //aux
                    c = new GurobiConstraint();
                    c.setName("d_ij:->:aux_X");
                    c.addToLHS(vp.aux_dOut_cqs.get(o)[4], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.x, 1.0);
                    c.addToRHS(vp.oCoordinate_iqs.get(o)[0], -1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_dOut_cqs.get(o)[0], vp.aux_dOut_cqs.get(o)[4], "abs_x_Out");
                    c = new GurobiConstraint();
                    c.setName("d_ij:->:aux_Y");
                    c.addToLHS(vp.aux_dOut_cqs.get(o)[5], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.y, 1.0);
                    c.addToRHS(vp.oCoordinate_iqs.get(o)[1], -1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_dOut_cqs.get(o)[1], vp.aux_dOut_cqs.get(o)[5], "abs_y_Out");
                    c = new GurobiConstraint();
                    c.setName("d_ij:->:aux_XY");
                    c.addToLHS(vp.aux_dOut_cqs.get(o)[6], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_dOut_cqs.get(o)[0], 1.0);
                    c.addToRHS(vp.aux_dOut_cqs.get(o)[1], -1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_dOut_cqs.get(o)[3], vp.aux_dOut_cqs.get(o)[6], "abs(abs_x-abs_y)");
                    executor.addGenConstraintMin(vp.aux_dOut_cqs.get(o)[2], new GurobiVariable[]{vp.aux_dOut_cqs.get(o)[0], vp.aux_dOut_cqs.get(o)[1]}, (double) 2 * M, "min_Out");

                    //d_ij: m->n (pl.2)
                    for (Obstacle on : obstacles) {
                        if (!o.getName().equals(on.getName())) {
                            c = new GurobiConstraint();
                            c.setName("d_ij: m->n (pl.2)");
                            c.addToLHS(vp.dOmOn_cq.get(o).get(on), 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.aux_dOmOn_cqs.get(o).get(on)[2], Math.sqrt(2));
                            c.addToRHS(vp.aux_dOmOn_cqs.get(o).get(on)[3], 1.0);
                            c.addToRHS(vp.omOnCnn_q.get(o).get(on), M);
                            c.setRHSConstant(-M);
                            executor.addConstraint(c);
                            //auxiliary
                            c = new GurobiConstraint();
                            c.setName("d_ij: m->n:aux_X");
                            c.addToLHS(vp.oCoordinate_iqs.get(o)[0], 1.0);
                            c.addToLHS(vp.oCoordinate_iqs.get(on)[0], -1.0);
                            c.setSense('=');
                            c.addToRHS(vp.aux_dOmOn_cqs.get(o).get(on)[4], 1.0);
                            executor.addConstraint(c);
                            executor.addGenConstraintAbs(vp.aux_dOmOn_cqs.get(o).get(on)[0], vp.aux_dOmOn_cqs.get(o).get(on)[4], "absX_dOmOn");
                            c = new GurobiConstraint();
                            c.setName("d_ij: m->n:aux_Y");
                            c.addToLHS(vp.oCoordinate_iqs.get(o)[1], 1.0);
                            c.addToLHS(vp.oCoordinate_iqs.get(on)[1], -1.0);
                            c.setSense('=');
                            c.addToRHS(vp.aux_dOmOn_cqs.get(o).get(on)[5], 1.0);
                            executor.addConstraint(c);
                            executor.addGenConstraintAbs(vp.aux_dOmOn_cqs.get(o).get(on)[1], vp.aux_dOmOn_cqs.get(o).get(on)[5], "absY_dOmOn");
                            c = new GurobiConstraint();
                            c.setName("d_ij: m->n:aux_XY");
                            c.addToLHS(vp.aux_dOmOn_cqs.get(o).get(on)[0], 1.0);
                            c.addToLHS(vp.aux_dOmOn_cqs.get(o).get(on)[1], -1.0);
                            c.setSense('=');
                            c.addToRHS(vp.aux_dOmOn_cqs.get(o).get(on)[6], 1.0);
                            executor.addConstraint(c);
                            executor.addGenConstraintAbs(vp.aux_dOmOn_cqs.get(o).get(on)[3], vp.aux_dOmOn_cqs.get(o).get(on)[6], "absXY_dOmOn");
                            executor.addGenConstraintMin(vp.aux_dOmOn_cqs.get(o).get(on)[2], new GurobiVariable[]{vp.aux_dOmOn_cqs.get(o).get(on)[0], vp.aux_dOmOn_cqs.get(o).get(on)[1]}, (double) M, "min_dOmOn");

                        }
                    }
                    //d_ij: <-
                    c = new GurobiConstraint();
                    c.setName("d_ij: <-");
                    c.addToLHS(vp.dInOut_cqs[1], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dIn_cqs.get(o)[2], Math.sqrt(2));
                    c.addToRHS(vp.aux_dIn_cqs.get(o)[3], 1.0);
                    c.addToRHS(vp.inOutCnn_qs.get(o)[1], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    //auxiliary
                    c = new GurobiConstraint();
                    c.setName("d_ij: <-:aux_X");
                    c.addToLHS(vpN.x, 1.0);
                    c.addToLHS(vp.oCoordinate_iqs.get(o)[0], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_dIn_cqs.get(o)[4], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_dIn_cqs.get(o)[0], vp.aux_dIn_cqs.get(o)[4], "absX_dIn");
                    c = new GurobiConstraint();
                    c.setName("d_ij: <-:aux_Y");
                    c.addToLHS(vpN.y, 1.0);
                    c.addToLHS(vp.oCoordinate_iqs.get(o)[1], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_dIn_cqs.get(o)[5], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_dIn_cqs.get(o)[1], vp.aux_dIn_cqs.get(o)[5], "absY_dIn");
                    c = new GurobiConstraint();
                    c.setName("d_ij: <-:aux_XY");
                    c.addToLHS(vp.aux_dIn_cqs.get(o)[0], 1.0);
                    c.addToLHS(vp.aux_dIn_cqs.get(o)[1], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_dIn_cqs.get(o)[6], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_dIn_cqs.get(o)[3], vp.aux_dIn_cqs.get(o)[6], "absXY_dIn");
                    executor.addGenConstraintMin(vp.aux_dIn_cqs.get(o)[2], new GurobiVariable[]{vp.aux_dIn_cqs.get(o)[0], vp.aux_dIn_cqs.get(o)[1]}, (double)M, "min_dIn");




                }


            }

            /*
            Index 0 -- vps.size() - 1
             */
            if (virtualPointVars.indexOf(vp) < virtualPointVars.size() - 1) {
                vpN = virtualPointVars.get(virtualPointVars.indexOf(vp) + 1);

                /*
                d_ij without Detour
                VVVV
                 */
                c = new GurobiConstraint();
                c.setName("d_ij:woDetour");
                c.addToLHS(vp.dist_cqs[0], 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_dist_cqs[2], Math.sqrt(2));
                c.addToRHS(vp.aux_dist_cqs[3], 1.0);
                executor.addConstraint(c);
                //aux_ABS
                c = new GurobiConstraint();
                c.setName("d_ij:woDetour:aux_x");
                c.addToLHS(vp.x, 1.0);
                c.addToLHS(vpN.x, -1.0);
                c.setSense('=');
                c.addToRHS(vp.aux_dist_cqs[4], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_dist_cqs[0], vp.aux_dist_cqs[4], "|vi.x-vj.x|_" + virtualPointVars.indexOf(vp));
                c = new GurobiConstraint();
                c.setName("d_ij:woDetour:aux_Y");
                c.addToLHS(vp.y, 1.0);
                c.addToLHS(vpN.y, -1.0);
                c.setSense('=');
                c.addToRHS(vp.aux_dist_cqs[5], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_dist_cqs[1], vp.aux_dist_cqs[5], "|vi.y-j.y|_" + virtualPointVars.indexOf(vp));
                executor.addGenConstraintMin(vp.aux_dist_cqs[2], new GurobiVariable[]{vp.aux_dist_cqs[0], vp.aux_dist_cqs[1]}, (double) M, "min(|vi.x-vj.x|,|vi.y-j.y|)_" + virtualPointVars.indexOf(vp));
                c = new GurobiConstraint();
                c.setName("d_ij:woDetour:aux_XY");
                c.addToLHS(vp.aux_dist_cqs[0], 1.0);
                c.addToLHS(vp.aux_dist_cqs[1], -1.0);
                c.setSense('=');
                c.addToRHS(vp.aux_dist_cqs[6], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_dist_cqs[3], vp.aux_dist_cqs[6], "||vi.x-vj.x|-|vi.y-j.y||_" + virtualPointVars.indexOf(vp));
                /*
                AAAA
                d_ij
                 */

                //busLength
                c_busLength.addToRHS(vp.dist_cqs[0], 1.0);




            }



            /*
            Connection with Slaves
             */
            GurobiConstraint c_vsCnn = new GurobiConstraint();
            c_vsCnn.setSense('=');
            c_vsCnn.setRHSConstant(1.0);
            executor.addConstraint(c_vsCnn);
            for (PseudoBase sv : slaves) {
                //sv.1
                c_vsCnn.addToLHS(vp.vsCnn_q.get(sv), 1.0);


                /*
                d_is_j
                VVVV
                 */
                c = new GurobiConstraint();
                c.setName("d_is_j_woDetour");
                c.addToLHS(vp.vs_dist_cq.get(sv), 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_vsDist_cqs.get(sv)[2], Math.sqrt(2));
                c.addToRHS(vp.aux_vsDist_cqs.get(sv)[3], 1.0);
                executor.addConstraint(c);
                //aux
                c = new GurobiConstraint();
                c.setName("d_is_j_woDetour:aux_X");
                c.addToLHS(vp.x, 1.0);
                c.setLHSConstant(-sv.getX());
                c.setSense('=');
                c.addToRHS(vp.aux_vsDist_cqs.get(sv)[4], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("d_is_j_woDetour:aux_Y");
                c.addToLHS(vp.y, 1.0);
                c.setLHSConstant(-sv.getY());
                c.setSense('=');
                c.addToRHS(vp.aux_vsDist_cqs.get(sv)[5], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_vsDist_cqs.get(sv)[0], vp.aux_vsDist_cqs.get(sv)[4], "|vi.x-sj.x|" + "v" + virtualPointVars.indexOf(vp) + "->" + sv.getName());
                executor.addGenConstraintAbs(vp.aux_vsDist_cqs.get(sv)[1], vp.aux_vsDist_cqs.get(sv)[5], "|vi.y-sj.y|" + "v" + virtualPointVars.indexOf(vp) + "->" + sv.getName());
                executor.addGenConstraintMin(vp.aux_vsDist_cqs.get(sv)[2], new GurobiVariable[]{vp.aux_vsDist_cqs.get(sv)[0], vp.aux_vsDist_cqs.get(sv)[1]}, (double) M, "min(|vi.x-sj.x|,|vi.y-sj.y|)" + "v" + virtualPointVars.indexOf(vp) + "->" + sv.getName());
                c = new GurobiConstraint();
                c.setName("d_is_j_woDetour:aux_XY");
                c.addToLHS(vp.aux_vsDist_cqs.get(sv)[0], 1.0);
                c.addToLHS(vp.aux_vsDist_cqs.get(sv)[1], -1.0);
                c.setSense('=');
                c.addToRHS(vp.aux_vsDist_cqs.get(sv)[6], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_vsDist_cqs.get(sv)[3], vp.aux_vsDist_cqs.get(sv)[6], "||vi.x-sj.x|-|vi.y-sj.y||" + "v" + virtualPointVars.indexOf(vp) + "->" + sv.getName());
                /*
                AAAA
                d_i_sj
                 */

                //sv.3
//                c = new GurobiConstraint();
//                c.setName("sv.3_1");
//                c.addToLHS(vp.dist_cqs[1], 1.0);
//                c.setSense('<');
//                c.addToRHS(vp.vs_dist_cq.get(sv), 1.0);
//                c.addToRHS(vp.vsCnn_q.get(sv), -M);
//                c.setRHSConstant(M);
//                executor.addConstraint(c);
//                c = new GurobiConstraint();
//                c.setName("sv.3_2");
//                c.addToLHS(vp.dist_cqs[1], 1.0);
//                c.setSense('>');
//                c.addToRHS(vp.vs_dist_cq.get(sv), 1.0);
//                c.addToRHS(vp.vsCnn_q.get(sv), M);
//                c.setRHSConstant(-M);
//                executor.addConstraint(c);
                qc = new GurobiQuadConstraint();
                qc.setName("sv.3");
                qc.addToLHS(vp.dist_cqs[1], 1.0);
                qc.setSense('>');
                qc.addToRHS(vp.vs_dist_cq.get(sv), vp.vsCnn_q.get(sv), 1.0);
                executor.addConstraint(qc);


            }


        }


    }


    public void buildVars(ArrayList<Obstacle> obstacles, ArrayList<VirtualPointVar> virtualPointVars, ArrayList<PseudoBase> slaves, PseudoBase master) {
        GurobiVariable[] qs;
        GurobiVariable q, iq, cq;

        /*
        find the boundary of the board
         */
        ArrayList<Integer> xs = new ArrayList<>();
        ArrayList<Integer> ys = new ArrayList<>();
        for (Obstacle o : obstacles) {
            xs.add(o.getMinX());
            xs.add(o.getMaxX());
            ys.add(o.getMinY());
            ys.add(o.getMaxY());
        }
        for (PseudoBase sv : slaves) {
            xs.add(sv.getX());
            ys.add(sv.getY());
        }
        xs.add(master.getX());
        ys.add(master.getY());

        int lb_x = Collections.min(xs);
        int ub_x = Collections.max(xs);
        int lb_y = Collections.min(ys);
        int ub_y = Collections.max(ys);
        //lb_x
        if (lb_x <= 0) {
            lb_x *= 2;
        } else {
            lb_x *= 0.5;
        }
        //lb_y
        if (lb_y <= 0) {
            lb_y *= 2;
        } else {
            lb_y *= 0.5;
        }
        //ub_x
        if (ub_x >= 0) {
            ub_x *= 2;
        } else {
            ub_x *= 0.5;
        }
        //ub_y
        if (ub_y >= 0) {
            ub_y *= 2;
        } else {
            ub_y *= 0.5;
        }

        int lb = Math.min(lb_x, lb_y);
        int ub = Math.max(ub_x, ub_y);



        /*
        initialize virtual points
         */
        for (int i = 0; i < slaves.size(); ++i) {
            VirtualPointVar vp = new VirtualPointVar();
            virtualPointVars.add(vp);
            vp.x = new GurobiVariable(GRB.INTEGER, -M, M, "x" + i);
            executor.addVariable(vp.x);
            vp.y = new GurobiVariable(GRB.INTEGER, -M, M, "y" + i);
            executor.addVariable(vp.y);


            for (Obstacle o : obstacles) {
                /*
                Non-overlapping
                0: nonL
                1: nonR
                2: nonA
                3: nonB
                 */
                vp.non_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_non_qs_", 4));

                /*
                rel_qs
                0: ul
                1: ur
                2: lr
                3: ll
                4: d
                5: u
                 */
                vp.rel_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_rel_qs_", 6));

                /*
                diagonal_qs
                0: o_tL
                1: o_tR
                2: o_bL
                3: o_bR
                 */
                vp.diagonal_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_diagonal_qs_", 4));

                /*
                o_vp_relObstacles_qs
                0: ul->lr
                1: lr->ul
                2: ur->ll
                3: ll->ur
                4: relative obstacle: aux.2
                 */
                vp.relObstacles_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_relObstacles_qs_", 5));


                /*
                corner_qs
                0: ll
                1: ur
                2: ul
                3: lr
                 */
                vp.corner_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_corner_qs_", 4));

                /*
                inOutCnn_qs
                0: ->
                1: <-
                */
                vp.inOutCnn_qs.put(o, buildBinaryVar("v_" + i + ";" + o.getName() + "_inOutCnn_qs_", 2));


                /*
                oCoordinate_iqs
                0: x_m
                1: y_m
                 */
                vp.oCoordinate_iqs.put(o, buildIntVar(-M, M, "v_" + i + ";" + o.getName() + "_oCoordinate_iqs_", 2));



                /*
                omOnCnn_q
                0: q_ij^m->n
                 */
                Map<Obstacle, GurobiVariable> omOnCnn_qMap = new HashMap<>();
                /*
                dOmOn_cqMap
                0: d_m->n
                 */
                Map<Obstacle, GurobiVariable> dOmOn_cqMap = new HashMap<>();
                /*
                Auxiliary absolute values: aux_dOmOn_cqs
                0: |x_m - x_n|
                1: |y_m - y_n|
                2: min(|x_m - x_n|, |y_m - y_n|)
                3: ||x_m - x_n| - |y_m - y_n||
                4: x_m - x_n
                5: y_m - y_n
                6: |x_m - x_n| - |y_m - y_n|
                 */
                Map<Obstacle, GurobiVariable[]> aux_dOmOn_cqsMap = new HashMap<>();
                for (Obstacle other_o : obstacles) {

                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + o.getName() + "-" + other_o.getName() + "_omOnCnn_q_");
                    executor.addVariable(q);
                    omOnCnn_qMap.put(other_o, q);

                    cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "v_" + i + ";" + o.getName() + "->" + other_o.getName() + "_dOmOn_cq_");
                    executor.addVariable(cq);
                    dOmOn_cqMap.put(other_o, cq);

                    aux_dOmOn_cqsMap.put(other_o, buildContinuousVar(0, M, "v_" + i + ";" + o.getName() + "->" + other_o.getName() + "_aux_dOmOn_cqs_", 7));
                }
                vp.omOnCnn_q.put(o, omOnCnn_qMap);
                vp.dOmOn_cq.put(o, dOmOn_cqMap);
                vp.aux_dOmOn_cqs.put(o, aux_dOmOn_cqsMap);


                /*
                Auxiliary absolute values: aux_dOut_cqs
                0: |vi.x - x_m|
                1: |vi.y - y_m|
                2: min(|vi.x - x_m|, |vi.y - y_m|)
                3: ||vi.x - x_m| - |vi.y - y_m||
                4: vi.x - x_m
                5: vi.y - y_m
                6: |vi.x - x_m| - |vi.y - y_m|
                 */
                vp.aux_dOut_cqs.put(o, buildContinuousVar(0, M, "v_" + i + ";" + o.getName() + "_aux_dOut_cqs_", 7));

                /*
                Auxiliary absolute values: aux_dIn_cqs
                0: |vi.x - x_m|
                1: |vi.y - y_m|
                2: min(|vi.x - x_m|, |vi.y - y_m|)
                3: ||vi.x - x_m| - |vi.y - y_m||
                4: vi.x - x_m
                5: vi.y - y_m
                6: |vi.x - x_m| - |vi.y - y_m|
                 */
                vp.aux_dIn_cqs.put(o, buildContinuousVar(0, M, "v_" + i + ";" + o.getName() + "_aux_dIn_cqs_", 7));

            }

            /*
            detour_qs
            0: ul->lr
            1: lr->ul
            2: ur->ll
            3: ll->ur
            4: detour trigger: aux.3
             */
            vp.detour_qs = buildBinaryVar("v" + i + "_detour_qs_", 5);

            /*
            dInOut_cqs
            0: d_->
            1: d_<-
             */
            vp.dInOut_cqs = buildContinuousVar(0, M, "v" + i + "_dInOut_cqs_", 2);

            /*
            dist_cqs
            0: vv dist
            1: v.corrS dist
            2: vm dist (only for 1st vp)
             */
            if (i == 0) {
                vp.dist_cqs = buildContinuousVar(0, M, "v" + i + "dist_cqs", 3);
            } else {
                vp.dist_cqs = buildContinuousVar(0, M, "v" + i + "dist_cqs", 2);
            }

            /*
            Auxiliary absolute values: aux_dist_cqs
            0: |vi.x-vj.x|
            1: |vi.y-j.y|
            2: min(|vi.x-vj.x|,|vi.y-j.y|)
            3: ||vi.x-vj.x|-|vi.y-j.y||
            4: vi.x-vj.x
            5: vi.y-j.y
            6: |vi.x-vj.x|-|vi.y-j.y|
             */
            vp.aux_dist_cqs = buildContinuousVar(0, M, "v" + i + "_aux_dist_cqs_", 7);





            /*
            Regarding Slaves
             */
            for (PseudoBase sv : slaves) {
                //vsCnn_q
                q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_vsCnn_q_" + sv.getName());
                executor.addVariable(q);
                vp.vsCnn_q.put(sv, q);

                /*
                vs_relObstacles_qs
                0: ul->lr
                1: lr->ul
                2: ur->ll
                3: ll->ur
                4: relative obstacle: aux.2
                */
                Map<Obstacle, GurobiVariable[]> vs_relObstacles_qsMap = new HashMap<>();
                /*
                vs_corner_qs
                0: ll
                1: ur
                2: ul
                3: lr
                 */
                Map<Obstacle, GurobiVariable[]> vs_corner_qsMap = new HashMap<>();
                /*
                vs_omOnCnn_q
                q_i_sj^m->n
                 */
                Map<Obstacle, Map<Obstacle, GurobiVariable>> vs_omOnCnn_qMap = new HashMap<>();
                /*
                vs_inCnn_q
                sj<-
                 */
                Map<Obstacle, GurobiVariable> vs_inCnn_qMap = new HashMap<>();
                /*
                vs_oCoordinate_iqs
                0: x_m
                1: y_m
                */
                Map<Obstacle, GurobiVariable[]> vs_oCoordinate_iqsMap = new HashMap<>();
                /*
                vs_dOmOn_cqs
                0: d_m->n
                 */
                Map<Obstacle, Map<Obstacle, GurobiVariable[]>> vs_dOmOn_cqsMap = new HashMap<>();


                for (Obstacle o : obstacles) {
                    vs_relObstacles_qsMap.put(o, buildBinaryVar("v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_relObstacles_qs_", 5));
                    vs_corner_qsMap.put(o, buildBinaryVar("v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_corner_qs_", 4));


                    Map<Obstacle, GurobiVariable> vs_onCnn_qMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable[]> vs_dOn_cqsMap = new HashMap<>();
                    for (Obstacle other_o : obstacles) {
                        q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_omOnCnn_q");
                        executor.addVariable(q);
                        vs_onCnn_qMap.put(other_o, q);

                        vs_dOn_cqsMap.put(other_o, buildContinuousVar(0, M, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_dOmOn_cqs_", 1));
                    }
                    vs_omOnCnn_qMap.put(o, vs_onCnn_qMap);

                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_inCnn_q");
                    executor.addVariable(q);
                    vs_inCnn_qMap.put(o, q);

                    vs_oCoordinate_iqsMap.put(o, buildIntVar(-M, M, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_oCoordinate_iqs_", 2));
                    vs_dOmOn_cqsMap.put(o, vs_dOn_cqsMap);
                }
                vp.vs_relObstacles_qs.put(sv, vs_relObstacles_qsMap);
                vp.vs_corner_qs.put(sv, vs_corner_qsMap);
                vp.vs_omOnCnn_q.put(sv, vs_omOnCnn_qMap);
                vp.vs_inCnn_q.put(sv, vs_inCnn_qMap);
                vp.vs_oCoordinate_iqs.put(sv, vs_oCoordinate_iqsMap);
                vp.vs_dOmOn_cqs.put(sv, vs_dOmOn_cqsMap);


                /*
                vs_detour_qs
                0: ul->lr
                1: lr->ul
                2: ur->ll
                3: ll->ur
                4: q_ij^d: detour trigger: aux.3
                */
                vp.vs_detour_qs.put(sv, buildBinaryVar("v" + i + ";" + sv.getName() + "_vs_detour_qs_", 5));

                /*
                vs_dIn_cqs
                0: d_<-
                 */
                vp.vs_dIn_cqs.put(sv, buildContinuousVar(0, M, "v" + i + ";" + sv.getName() + "_vs_dIn_cqs_", 1));

                /*
                Auxiliary absolute values: vs
                0: |vi.x-sj.x|
                1: |vi.y-sj.y|
                2: min(|vi.x-sj.x|,|vi.y-sj.y|)
                3: ||vi.x-sj.x|-|vi.y-sj.y||
                4: vi.x-sj.x
                5: vi.y-sj.y
                6: |vi.x-sj.x|-|vi.y-sj.y|
                 */
                vp.aux_vsDist_cqs.put(sv, buildContinuousVar(0, M, "v" + i + ";" + sv.getName() + "_aux_vsDist_cqs_", 7));

                /*
                d_i_sj
                 */
                cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "v" + i + ";" + sv.getName() + "_d_i_sj");
                executor.addVariable(cq);
                vp.vs_dist_cq.put(sv, cq);
            }

        }


    }


    private GurobiVariable[] buildContinuousVar(int lb, int ub, String varName, int varCnt) {
        GurobiVariable[] qs = new GurobiVariable[varCnt];
        for (int var_cnt = 0; var_cnt < varCnt; ++var_cnt) {
            qs[var_cnt] = new GurobiVariable(GRB.CONTINUOUS, lb, ub, varName + var_cnt);
            executor.addVariable(qs[var_cnt]);
        }
        return qs;
    }

    private GurobiVariable[] buildBinaryVar(String varName, int varCnt) {
        GurobiVariable[] qs = new GurobiVariable[varCnt];
        for (int var_cnt = 0; var_cnt < varCnt; ++var_cnt) {
            qs[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, varName + var_cnt);
            executor.addVariable(qs[var_cnt]);
        }
        return qs;
    }

    private GurobiVariable[] buildIntVar(int lb, int ub, String varName, int varCnt) {
        GurobiVariable[] qs = new GurobiVariable[varCnt];
        for (int var_cnt = 0; var_cnt < varCnt; ++var_cnt) {
            qs[var_cnt] = new GurobiVariable(GRB.INTEGER, lb, ub, varName + var_cnt);
            executor.addVariable(qs[var_cnt]);
        }
        return qs;
    }


    public OppositeType relationDetermineRegardOs(ArrayList<Obstacle> relevantObstacles, PseudoBase cur_n, PseudoBase other_n) {

        OppositeType type = OppositeType.NoOppositeRelation;
        for (Obstacle cur_o : relevantObstacles) {
            for (Obstacle other_o : relevantObstacles) {
                //topL to bottomR
                if (cur_o.topL_bottomR_oo(other_o) && cur_n.getPseudo_oRel_qs().get(cur_o)[4] == 1 && other_n.getPseudo_oRel_qs().get(other_o)[7] == 1) {
                    type = OppositeType.topLtoBottomR;
                    break;
                }
                //bottomR to topL
                if (cur_o.bottomR_topL_oo(other_o) && cur_n.getPseudo_oRel_qs().get(cur_o)[7] == 1 && other_n.getPseudo_oRel_qs().get(other_o)[4] == 1) {
                    type = OppositeType.bottomRtoTopL;
                    break;
                }
                //topR to bottomL
                if (cur_o.topR_bottomL_oo(other_o) && cur_n.getPseudo_oRel_qs().get(cur_o)[5] == 1 && other_n.getPseudo_oRel_qs().get(other_o)[6] == 1) {
                    type = OppositeType.topRtoBottomL;
                    break;
                }
                //bottomL to topR
                if (cur_o.bottomL_topR_oo(other_o) && cur_n.getPseudo_oRel_qs().get(cur_o)[6] == 1 && other_n.getPseudo_oRel_qs().get(other_o)[5] == 1) {
                    type = OppositeType.bottomLtoTopR;
                }


            }
        }


        return type;
    }

    /**
     * Sorted ArrayList of Obstacles according to the SortType
     *
     * @param targetObstacles ArrayList of Obstacles
     * @param type            SortType
     */
    public void sortedObstacles(ArrayList<Obstacle> targetObstacles, SortType type) {

        if (type == SortType.LeftToRight) {
            for (int i = 0; i < targetObstacles.size(); ++i) {

                boolean flag = true;
                while (flag) {
                    Obstacle cur_o = targetObstacles.get(i);
                    for (int j = i + 1; j < targetObstacles.size(); ++j) {
                        Obstacle other_o = targetObstacles.get(j);
                        if (cur_o.onLeft(other_o)) {
                            Collections.swap(targetObstacles, i, j);
                            break;
                        }
                        if (j == targetObstacles.size() - 1) {
                            flag = false;
                        }
                    }
                    if (i == targetObstacles.size() - 1) flag = false;
                }
            }

        } else if (type == SortType.RightToLeft) {

            for (int i = 0; i < targetObstacles.size(); ++i) {

                boolean flag = true;
                while (flag) {
                    Obstacle cur_o = targetObstacles.get(i);
                    for (int j = i + 1; j < targetObstacles.size(); ++j) {
                        Obstacle other_o = targetObstacles.get(j);
                        if (cur_o.onRight(other_o)) {
                            Collections.swap(targetObstacles, i, j);
                            break;
                        }
                        if (j == targetObstacles.size() - 1) {
                            flag = false;
                        }
                    }
                    if (i == targetObstacles.size() - 1) flag = false;
                }
            }
        } else if (type == SortType.TopToBottom) {

            for (int i = 0; i < targetObstacles.size(); ++i) {

                boolean flag = true;
                while (flag) {
                    Obstacle cur_o = targetObstacles.get(i);
                    for (int j = i + 1; j < targetObstacles.size(); ++j) {
                        Obstacle other_o = targetObstacles.get(j);
                        if (cur_o.onTop(other_o)) {
                            Collections.swap(targetObstacles, i, j);
                            break;
                        }
                        if (j == targetObstacles.size() - 1) {
                            flag = false;
                        }
                    }
                    if (i == targetObstacles.size() - 1) flag = false;
                }
            }
        } else if (type == SortType.BottomToTop) {

            for (int i = 0; i < targetObstacles.size(); ++i) {

                boolean flag = true;
                while (flag) {
                    Obstacle cur_o = targetObstacles.get(i);
                    for (int j = i + 1; j < targetObstacles.size(); ++j) {
                        Obstacle other_o = targetObstacles.get(j);
                        if (cur_o.onBottom(other_o)) {
                            Collections.swap(targetObstacles, i, j);
                            break;
                        }
                        if (j == targetObstacles.size() - 1) {
                            flag = false;
                        }
                    }
                    if (i == targetObstacles.size() - 1) flag = false;
                }
            }
        }


    }

    private ArrayList<PseudoBase> parallelogramClockwise(PseudoBase cur_node, PseudoBase other_node) {
        PseudoBase p1 = cur_node;
        PseudoBase p3 = other_node;
        PseudoBase p2 = null, p4 = null;
        int dx = Math.abs(cur_node.getX() - other_node.getX());
        int dy = Math.abs(cur_node.getY() - other_node.getY());
        if (dx == dy || cur_node.getX() == other_node.getX() || cur_node.getY() == other_node.getY()) {
            p2 = cur_node;
            p4 = other_node;
        } else {
            int plusDY = (int) Math.max(dy * Math.signum(dx - dy), 0);
            int plusDx = (int) Math.max(dx * Math.signum(dy - dx), 0);
            //Case1: other -> lowerRight
            if (other_node.getX() > cur_node.getX() && other_node.getY() < cur_node.getY()) {
                p2 = new PseudoBase(other_node.getX() - plusDY, cur_node.getY() - plusDx);
                p4 = new PseudoBase(cur_node.getX() + plusDY, other_node.getY() - plusDx);
            }
            //Case2: other -> upperRight
            else if (other_node.getX() > cur_node.getX() && other_node.getY() > cur_node.getY()) {
                p2 = new PseudoBase(cur_node.getX() + plusDY, other_node.getY() - plusDx);
                p4 = new PseudoBase(other_node.getX() - plusDY, cur_node.getY() + plusDx);
            }
            //Case3: other -> upperLeft
            else if (other_node.getX() < cur_node.getX() && other_node.getY() > cur_node.getY()) {
                p4 = new PseudoBase(cur_node.getX() - plusDY, other_node.getY() - plusDx);
                p2 = new PseudoBase(other_node.getX() + plusDY, cur_node.getY() + plusDx);
            }
            //Case 4: other -> lowerLeft
            else if (other_node.getX() < cur_node.getX() && other_node.getY() < cur_node.getY()) {
                p4 = new PseudoBase(other_node.getX() + plusDY, cur_node.getY() - plusDx);
                p2 = new PseudoBase(cur_node.getX() - plusDY, other_node.getY() + plusDx);
            } else {
                System.err.println("Another Type of Parallelogram!!");
                System.out.println("cur_node = (" + cur_node.getX() + ", " + cur_node.getY() + "), other_node = (" + other_node.getX() + ", " + other_node.getY() + ")");
            }

        }
        ArrayList<PseudoBase> array = new ArrayList<PseudoBase>(Arrays.asList(p1, p2, p3, p4));
        return array;
    }

    private ArrayList<Obstacle> overlappedObstacles(ArrayList<Obstacle> obstacles, PseudoBase cur_node, PseudoBase other_node, PseudoBase bypassNode, Obstacle cur_o, Obstacle other_o) {

//        PseudoBase p1 = cur_node;
//        PseudoBase p3 = other_node;
//        PseudoBase p2 = null, p4 = null;
//        int dx = Math.abs(cur_node.getX() - other_node.getX());
//        int dy = Math.abs(cur_node.getY() - other_node.getY());
//        if (dx == dy || cur_node.getX() == other_node.getX() || cur_node.getY() == other_node.getY()) {
//            p2 = cur_node;
//            p4 = other_node;
//        } else {
//            int plusDY = (int) Math.max(dy * Math.signum(dx - dy), 0);
//            int plusDx = (int) Math.max(dx * Math.signum(dy - dx), 0);
//            //Case1: other -> lowerRight
//            if (other_node.getX() > cur_node.getX() && other_node.getY() < cur_node.getY()) {
//                p2 = new PseudoBase(other_node.getX() - plusDY, cur_node.getY() - plusDx);
//                p4 = new PseudoBase(cur_node.getX() + plusDY, other_node.getY() - plusDx);
//            }
//            //Case2: other -> upperRight
//            else if (other_node.getX() > cur_node.getX() && other_node.getY() > cur_node.getY()) {
//                p2 = new PseudoBase(cur_node.getX() + plusDY, other_node.getY() - plusDx);
//                p4 = new PseudoBase(other_node.getX() - plusDY, cur_node.getY() + plusDx);
//            }
//            //Case3: other -> upperLeft
//            else if (other_node.getX() < cur_node.getX() && other_node.getY() > cur_node.getY()) {
//                p4 = new PseudoBase(cur_node.getX() - plusDY, other_node.getY() - plusDx);
//                p2 = new PseudoBase(other_node.getX() + plusDY, cur_node.getY() + plusDx);
//            }
//            //Case 4: other -> lowerLeft
//            else if (other_node.getX() < cur_node.getX() && other_node.getY() < cur_node.getY()) {
//                p4 = new PseudoBase(other_node.getX() + plusDY, cur_node.getY() - plusDx);
//                p2 = new PseudoBase(cur_node.getX() - plusDY, other_node.getY() + plusDx);
//            } else {
//                System.err.println("Another Type of Parallelogram!!");
//                System.out.println("cur_node = (" + cur_node.getX() + ", " + cur_node.getY() + "), other_node = (" + other_node.getX() + ", " + other_node.getY() + ")");
//            }
//
//        }

        ArrayList<Obstacle> overlappedO = new ArrayList<>();

        for (Obstacle o : obstacles) {
            //check if one of the edges of obstacles v.s., one of the edges of the parallelogram
            if (segmentOverlappedObstacle(cur_node, bypassNode, o)) {
                overlappedO.add(o);
            }
            if (segmentOverlappedObstacle(bypassNode, other_node, o)) {
                overlappedO.add(o);
            }
        }


        //Left -- Right
        if (cur_node.getX() <= other_node.getX()) {
            if (cur_node.getType() == BaseType.lowerRight || cur_node.getType() == BaseType.upperRight) {
                overlappedO.remove(cur_o);
            }
            if (other_node.getType() == BaseType.lowerLeft || other_node.getType() == BaseType.upperLeft) {
                overlappedO.remove(other_o);
            }
        } else {
            if (cur_node.getType() == BaseType.lowerLeft || cur_node.getType() == BaseType.upperLeft) {
                overlappedO.remove(cur_o);
            }
            if (other_node.getType() == BaseType.lowerRight || other_node.getType() == BaseType.upperRight) {
                overlappedO.remove(other_o);
            }
        }
        //Up -- Down
        if (cur_node.getY() <= other_node.getY()) {
            if (cur_node.getType() == BaseType.upperLeft || cur_node.getType() == BaseType.upperRight) {
                overlappedO.remove(cur_o);
            }
            if (other_node.getType() == BaseType.lowerLeft || other_node.getType() == BaseType.lowerRight) {
                overlappedO.remove(other_o);
            }
        } else {
            if (cur_node.getType() == BaseType.lowerLeft || cur_node.getType() == BaseType.lowerRight) {
                overlappedO.remove(cur_o);
            }
            if (other_node.getType() == BaseType.upperLeft || other_node.getType() == BaseType.upperRight) {
                overlappedO.remove(other_o);
            }

        }

        return overlappedO;
    }

    private boolean segmentOverlappedObstacle(PseudoBase node1, PseudoBase node2, Obstacle o) {
        //node1_node2 vs o_LL_LR
        if (doIntersect(node1, node2, o.getLowerLeft(), o.getLowerRight())) {
            return true;
        }
        //node1_node2 vs o_LL_UL
        if (doIntersect(node1, node2, o.getLowerLeft(), o.getUpperLeft())) {
            return true;
        }
        //node1_node2 vs o_UR_UL
        if (doIntersect(node1, node2, o.getUpperRight(), o.getUpperLeft())) {
            return true;
        }
        //node1_node2 vs o_UR_LR
        return doIntersect(node1, node2, o.getUpperRight(), o.getLowerRight());
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
            basicBinaryVariables(master, o, slaves);
            for (PseudoBase slave : slaves) {
                basicBinaryVariables(slave, o, slaves);
            }

            /*
            oo_dir
             */
            for (Obstacle other_o : obstacles) {
                if (!other_o.getName().equals(o.getName())) {

                    //OtL:
                    if (o.atL(other_o)) {
                        o.addTotLObstacles(other_o);
                    }

                    //OtR:
                    if (o.atR(other_o)) {
                        o.addTotRObstacles(other_o);
                    }

                    //ObL:
                    if (o.abL(other_o)) {
                        o.addTobLObstacles(other_o);
                    }

                    //ObR:
                    if (o.abR(other_o)) {
                        o.addTobRObstacles(other_o);
                    }


                } else continue;
            }
        }


    }

    /**
     * Determine dir_q and rel_q
     *
     * @param base given point
     * @param o    given obstacle
     */
    private void basicBinaryVariables(PseudoBase base, Obstacle o, ArrayList<PseudoBase> bases) {

        /*
        oDir_q: L, R, A, B, UL, UR, LR, LL, D,U
         */
        int cnt_oDir_q = 6;
        int[] odir_q = new int[cnt_oDir_q];
        //UL
        if (base.getY() <= base.getX() + o.getMaxY() - o.getMinX()) {
            odir_q[0] = 1;
        } else odir_q[0] = 0;
        //UR
        if (base.getY() <= -base.getX() + o.getMaxY() + o.getMaxX()) {
            odir_q[1] = 1;
        } else odir_q[1] = 0;
        //LR
        if (base.getY() <= base.getX() + o.getMinY() - o.getMaxX()) {
            odir_q[2] = 1;
        } else odir_q[2] = 0;
        //LL
        if (base.getY() <= -base.getX() + o.getMinY() + o.getMinX()) {
            odir_q[3] = 1;
        } else odir_q[3] = 0;
        //D
        if (base.getY() <= (double) (o.getMinY() - o.getMaxY()) / (double) (o.getMaxX() - o.getMinX()) * (double) (base.getX() - o.getMinX()) + o.getMaxY()) {
            odir_q[4] = 1;
        } else odir_q[4] = 0;
        //U
        if (base.getY() <= (double) (o.getMaxY() - o.getMinY()) / (double) (o.getMaxX() - o.getMinX()) * (double) (base.getX() - o.getMinX()) + o.getMinY()) {
            odir_q[5] = 1;
        } else odir_q[5] = 0;
        base.addToPseudo_oDir_qs(o, odir_q);

        /*
        oRel_q:
         */
        int cnt_oRel_q = 4;
        int[] orel_q = new int[cnt_oRel_q];
        //UpperLeft
        if (odir_q[2] + odir_q[4] == 0 && odir_q[1] == 1) {
            orel_q[0] = 1;
            base.addToOULd(o);
        } else orel_q[0] = 0;
        //UpperRight
        if (odir_q[2] + odir_q[4] == 0 && odir_q[0] == 1) {
            orel_q[1] = 1;
            base.addToOURd(o);
        } else orel_q[1] = 0;
        //LowerLeft
        if (odir_q[0] + odir_q[4] == 2 && odir_q[2] == 0) {
            orel_q[2] = 1;
            base.addToOLLd(o);
        } else orel_q[2] = 0;
        //LowerRight
        if (odir_q[1] + odir_q[5] == 2 && odir_q[3] == 0) {
            orel_q[3] = 1;
            base.addToOLRd(o);
        } else orel_q[3] = 0;
        base.addToPseudo_oRel_qs(o, orel_q);


    }


    public static boolean onSegment(PseudoBase p, PseudoBase q, PseudoBase r) {
        return q.getX() <= Math.max(p.getX(), r.getX()) && q.getX() >= Math.min(p.getX(), r.getX())
                && q.getY() <= Math.max(p.getY(), r.getY()) && q.getY() >= Math.min(p.getY(), r.getY());
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


    public String convertGrbIntArrayToString(GurobiVariable[] grbIntArray) throws GRBException {
        StringBuilder grbAsString = new StringBuilder("[");
        for (GurobiVariable v : grbIntArray) {
            grbAsString.append(v.getIntResult() + " ");
        }
        grbAsString.delete(grbAsString.length()-1, grbAsString.length()).append("]");
        return grbAsString.toString();
    }

    public String convertGrbContArrayToString(GurobiVariable[] grbContArray) throws GRBException {
        StringBuilder grbAsString = new StringBuilder("[");
        for (GurobiVariable v : grbContArray) {
            grbAsString.append(v.getContResult() + " ");
        }
        grbAsString.delete(grbAsString.length()-1, grbAsString.length()).append("]");
        return grbAsString.toString();
    }

}
