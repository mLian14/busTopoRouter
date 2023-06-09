package processor;

import grb.*;
import gurobi.GRB;
import gurobi.GRBConstr;
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
    private final static int M = 99999;
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
        for (PseudoBase sv : slaves) {
            System.out.println(sv.getName() + "(" + sv.getX() + ", " + sv.getY() + ")");
            for (Obstacle o : obstacles) {
                System.out.print(o.getName() + "_dir:" + Arrays.toString(sv.getPseudo_oDir_qs().get(o)) + "|| ");
                System.out.println("_rel:" + Arrays.toString(sv.getPseudo_oRel_qs().get(o)) + "||");
            }
        }
        System.out.println("Master");
        for (Obstacle o : obstacles) {
            System.out.print(o.getName() + "_dir:" + Arrays.toString(master.getPseudo_oDir_qs().get(o)) + "|| ");
            System.out.println("_rel:" + Arrays.toString(master.getPseudo_oRel_qs().get(o)) + "||");
        }
        for (Obstacle o : obstacles) {
            System.out.println(o.getName());
            System.out.print("O_tL: ");
            for (Obstacle on : o.get_tLObstacles()) {
                System.out.print(on.getName() + "; ");
            }
            System.out.print("O_tR: ");
            for (Obstacle on : o.get_tRObstacles()) {
                System.out.print(on.getName() + "; ");
            }
            System.out.print("O_bL: ");
            for (Obstacle on : o.get_bLObstacles()) {
                System.out.print(on.getName() + "; ");
            }
            System.out.print("O_bR: ");
            for (Obstacle on : o.get_bRObstacles()) {
                System.out.print(on.getName() + "; ");
            }
            System.out.println();
        }


        executor = new GurobiExecutor("LinearBusRouting_KO");
        executor.setMIPGap(0);
        executor.setMIPGapAbs(0);
        executor.setTimeLimit(36000);
        //executor.setPresolve(0);


        //build Gurobi Variables
        GurobiVariable busLength, branchLength;
        busLength = new GurobiVariable(GRB.CONTINUOUS, 0, M, "busLength");
        executor.addVariable(busLength);
        branchLength = new GurobiVariable(GRB.CONTINUOUS, 0, M, "branchLength");
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

        //All Constraints
        /*for (GurobiConstraint c : executor.getConstraints()){
            System.out.println("C" + executor.getConstraints().indexOf(c));
            System.out.println(c);
        }*/

        executor.startOptimization();
        this.executor.getModel().write("debug.lp");
        int status = executor.getModel().get(GRB.IntAttr.Status);

        if (status == GRB.Status.UNBOUNDED) {
            System.out.println("The model cannot be solved " + "because it is unbounded");
        }
        else if (status == GRB.Status.OPTIMAL) {
            System.out.println("The optimal objective is " + executor.getModel().get(GRB.DoubleAttr.ObjVal));
        }
        else if (status != GRB.Status.INF_OR_UNBD && status != GRB.Status.INFEASIBLE) {
            System.out.println("Optimization was stopped with status " + status);
        }
        else if (executor.getModel().get(GRB.IntAttr.Status) == GRB.INFEASIBLE) {
            executor.whenInfeasibleWriteOutputTo("inf.ilp");
            System.out.println("HH:IIS");


            // Do IIS
            System.out.println("The model is infeasible; computing IIS");
            LinkedList<String> removed = new LinkedList<String>();

            // Loop until we reduce to a model that can be solved
            while (true) {
                executor.getModel().computeIIS();
                System.out.println("\nThe following constraint cannot be satisfied:");
                for (GRBConstr c : executor.getModel().getConstrs()) {
                    if (c.get(GRB.IntAttr.IISConstr) == 1) {
                        System.out.println(c.get(GRB.StringAttr.ConstrName));
                        // Remove a single constraint from the model
                        removed.add(c.get(GRB.StringAttr.ConstrName));
                        executor.getModel().remove(c);
                        break;
                    }
                }

                System.out.println();
                executor.getModel().optimize();
                status = executor.getModel().get(GRB.IntAttr.Status);

                if (status == GRB.Status.UNBOUNDED) {
                    System.out.println("The model cannot be solved because it is unbounded");
                    break;
                }
                if (status == GRB.Status.OPTIMAL) {
                    break;
                }
                if (status != GRB.Status.INF_OR_UNBD &&
                        status != GRB.Status.INFEASIBLE) {
                    System.out.println("Optimization was stopped with status " +
                            status);
                    break;
                }


            }
            System.out.println("\nThe following constraints were removed to get a feasible LP:");
            for (String s : removed) {
                System.out.print(s + " ");
            }
            System.out.println();
        }








        /*
        Retrieve
         */
        for (int i = 0; i<vps.size(); ++i){
            VirtualPointVar vp = vps.get(i);
            System.out.print("v" + i + ":");
            System.out.println("d_v->CoorS=" + vp.vs_corDist_cq.getContResult());
            for (PseudoBase sv : slaves){
                System.out.print(sv.getName() + ":_q_ij=" +vp.vsCnn_q.get(sv).getIntResult() + "; ");
                System.out.print("vs_dist:" + vp.vs_dist_cq.get(sv).getContResult() + "||");
            }
            System.out.println();
        }
        System.out.println("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");

        for (int i = 0; i < vps.size(); ++i) {
            VirtualPointVar vp = vps.get(i);
            System.out.println("vp" + i + "=(" + vp.x.getIntResult() + ", " + vp.y.getIntResult() + ").");
            if (i == 0){
                System.out.println("Master:");
                System.out.print("VM:detour_qs:" + convertGrbIntArrayToString(vp.vm_detour_qs) + "|| ");
                System.out.println("VM_dist_cq:" + vp.vm_dist_cq.getContResult() + "|| ");
            }


            System.out.print("detour_qs:" + convertGrbIntArrayToString(vp.detour_qs) + "|| ");
            System.out.println("vv_dist_cq:" + vp.dist_cq.getContResult() + "||");
            for (Obstacle o : obstacles) {
                System.out.print(o.getName() + " " + o.getMinX() + " " + o.getMaxX() + " " + o.getMinY() + " " + o.getMaxY() + "|| ");
                System.out.print("nonL:" + convertGrbIntArrayToString(vp.non_qs.get(o)) + "|| ");
                System.out.print("dir_qs:" + convertGrbIntArrayToString(vp.dir_qs.get(o)) + "|| ");
                System.out.print("rel_qs:" + convertGrbIntArrayToString(vp.rel_qs.get(o)) + "|| ");
                int itmp = 0;
                for (Obstacle on : o.get_tLObstacles()){
                    itmp += vp.rel_qs.get(on)[0].getIntResult();
                }
                System.out.print("_sum_tL:" + itmp +"|| ");
                itmp = 0;
                for (Obstacle on : o.get_tRObstacles()){
                    itmp += vp.rel_qs.get(on)[1].getIntResult();
                }
                System.out.print("_sum_tR:" + itmp +"|| ");
                itmp = 0;
                for (Obstacle on : o.get_bLObstacles()){
                    itmp += vp.rel_qs.get(on)[2].getIntResult();
                }
                System.out.print("_sum_bL:" + itmp +"|| ");
                itmp = 0;
                for (Obstacle on : o.get_bRObstacles()){
                    itmp += vp.rel_qs.get(on)[3].getIntResult();
                }
                System.out.println("_sum_bR:" + itmp +"|| ");

            }
            if (vp.detour_qs[4].getIntResult() == 1) {
                System.out.println("vpNext_Detour:");
                for (Obstacle om : obstacles) {
                    System.out.println(om.getName() + ":" + convertGrbIntArrayToString(vp.relObstacles_qs.get(om)));
                }
                for (Obstacle om : obstacles) {
                    if (vp.inOutCnn_qs.get(om)[0].getIntResult() == 1) {
                        System.out.print("vs->: " + om.getName() + ":");
                        System.out.println("corner:" + convertGrbIntArrayToString(vp.corner_qs.get(om)) + "||");
                    }
                    if (vp.inOutCnn_qs.get(om)[1].getIntResult() == 1) {
                        System.out.print("vs<-:" + om.getName());
                        System.out.println("corner:" + convertGrbIntArrayToString(vp.corner_qs.get(om)) + "||");
                    }


                }
                for (Obstacle om : obstacles) {
                    for (Obstacle on : obstacles) {
                        if (vp.omOnCnn_q.get(om).get(on).getIntResult() == 1) {
                            System.out.print(om.getName() + "-> " + on.getName() + ":cnn" + "|| ");
                            System.out.print("cornerOm:" + convertGrbIntArrayToString(vp.corner_qs.get(om)) + "||cornerOn:" + convertGrbIntArrayToString(vp.corner_qs.get(on)) + "||");
                            System.out.println("vs_dOmOn=" + vp.dOmOn_cq.get(om).get(on).getContResult());
                        }
                    }
                }
                System.out.println();

            }


            //System.out.println("vp" + i + "->Slave Relevant VVVVVVVVVVVVVVVVVVVVVVVVVV");
            for (PseudoBase sv : slaves) {
                if (vp.vsCnn_q.get(sv).getIntResult() == 1) {
                    System.out.println("vp" + i + "=(" + vp.x.getIntResult() + ", " + vp.y.getIntResult() + ")->" + sv.getName() + ":(" + sv.getX() + ", " + sv.getY() + ")");
                    System.out.print("d_v->CorrS= " + vp.vs_corDist_cq.getContResult());
                    System.out.println("|| d_vi_sj = " + vp.vs_dist_cq.get(sv).getContResult());
                    double dtmp = Math.sqrt(2) * vp.aux_vsDist_iqs.get(sv)[2].getContResult() + vp.aux_vsDist_iqs.get(sv)[3].getContResult();
                    System.out.println("aux_vsDist_cqs=" + convertGrbContArrayToString(vp.aux_vsDist_iqs.get(sv)) + ": " + dtmp);
                    System.out.print("vs_detour_qs:" + convertGrbIntArrayToString(vp.vs_detour_qs.get(sv)) + "|| ");
                    System.out.println("vs_dInOut_cqs:" + convertGrbContArrayToString(vp.vs_dInOut_cqs.get(sv)) + "|| ");

                    System.out.println(sv.getName());
                    for (Obstacle om : obstacles) {
                        System.out.print(om.getName() + "_dir:" + Arrays.toString(sv.getPseudo_oDir_qs().get(om)) + "|| ");
                        System.out.print("_rel:" + Arrays.toString(sv.getPseudo_oRel_qs().get(om)) + "||");
                        int itmp = 0;
                        for (Obstacle on : om.get_tLObstacles()){
                            itmp += sv.getPseudo_oRel_qs().get(on)[0];
                        }
                        System.out.print("_vs_sum_tL:" + itmp +"|| ");
                        itmp = 0;
                        for (Obstacle on : om.get_tRObstacles()){
                            itmp += sv.getPseudo_oRel_qs().get(on)[1];
                        }
                        System.out.print("_vs_sum_tR:" + itmp +"|| ");
                        itmp = 0;
                        for (Obstacle on : om.get_bLObstacles()){
                            itmp += sv.getPseudo_oRel_qs().get(on)[2];
                        }
                        System.out.print("_vs_sum_bL:" + itmp +"|| ");
                        itmp = 0;
                        for (Obstacle on : om.get_bRObstacles()){
                            itmp += sv.getPseudo_oRel_qs().get(on)[3];
                        }
                        System.out.println("_vs_sum_bR:" + itmp +"|| ");

                    }

                    if (vp.vs_detour_qs.get(sv)[4].getIntResult() == 1) {
                        System.out.println("detour");
                        /*for (Obstacle om : obstacles){
                            System.out.println("vs_relObstacle:" + convertGrbIntArrayToString(vp.vs_relObstacles_qs.get(sv).get(om)));
                        }*/
                        for (Obstacle om : obstacles) {
                            System.out.println(om.getName() + ":" + convertGrbIntArrayToString(vp.vs_relObstacles_qs.get(sv).get(om)));
                        }
                        for (Obstacle om : obstacles) {
                            if (vp.vs_inOutCnn_qs.get(sv).get(om)[0].getIntResult() == 1) {
                                System.out.print("vs->: " + om.getName() + ":");
                                System.out.println("corner:" + convertGrbIntArrayToString(vp.vs_corner_qs.get(sv).get(om)) + "||");
                            }
                            if (vp.vs_inOutCnn_qs.get(sv).get(om)[1].getIntResult() == 1) {
                                System.out.print("vs<-:" + om.getName());
                                System.out.println("corner:" + convertGrbIntArrayToString(vp.vs_corner_qs.get(sv).get(om)) + "||");
                            }


                        }
                        for (Obstacle om : obstacles) {
                            for (Obstacle on : obstacles) {
                                if (vp.vs_omOnCnn_q.get(sv).get(om).get(on).getIntResult() == 1) {
                                    System.out.print(om.getName() + "-> " + on.getName() + ":cnn" + "|| ");
                                    System.out.print("cornerOm:" + convertGrbIntArrayToString(vp.vs_corner_qs.get(sv).get(om)) + "||cornerOn:" + convertGrbIntArrayToString(vp.vs_corner_qs.get(sv).get(on)) + "||");
                                    System.out.println("vs_dOmOn=" + vp.vs_dOmOn_cq.get(sv).get(om).get(on).getContResult());
                                }
                            }
                        }
                    }
                    System.out.println();
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

        // Dispose of model and environment
        executor.getModel().dispose();
        executor.getEnv().dispose();

        return output;
    }

    public void buildCons(ArrayList<Obstacle> obstacles, ArrayList<VirtualPointVar> virtualPointVars, ArrayList<PseudoBase> slaves, PseudoBase master, GurobiVariable busLength, GurobiVariable branchLength) throws GRBException {
        double eps = 1;
        double minDist = 1;

        GurobiConstraint c;
        GurobiQuadConstraint qc;

        VirtualPointVar vpN;
        int itmp;


        //busLength
        GurobiConstraint c_busLength = new GurobiConstraint();
        c_busLength.setName("cBusLength");
        c_busLength.addToLHS(busLength, 1.0);
        c_busLength.setSense('=');
        executor.addConstraint(c_busLength);
        //branchLength
        GurobiConstraint c_branchLength = new GurobiConstraint();
        c_branchLength.setName("cBranchLength");
        c_branchLength.addToLHS(branchLength, 1.0);
        c_branchLength.setSense('=');
        executor.addConstraint(c_branchLength);

        //sv.2

        for (PseudoBase sv : slaves) {
            c = new GurobiConstraint();
            for (VirtualPointVar vp : virtualPointVars) {
                c.addToLHS(vp.vsCnn_q.get(sv), 1.0);
            }
            c.setSense('=');
            c.setRHSConstant(1.0);
            executor.addConstraint(c);
        }


        for (int i = 0; i < virtualPointVars.size(); ++i) {

            VirtualPointVar vp = virtualPointVars.get(i);
            //branchLength
            c_branchLength.addToRHS(vp.vs_corDist_cq, 1.0);

            //VM_cnn.4
            GurobiQuadConstraint c_vmOut = new GurobiQuadConstraint();
            GurobiConstraint c_vmULtoLR_geq = new GurobiConstraint();
            GurobiConstraint c_vmULtoLR_leq = new GurobiConstraint();
            GurobiConstraint c_vmLRtoUL_geq = new GurobiConstraint();
            GurobiConstraint c_vmLRtoUL_leq = new GurobiConstraint();
            GurobiConstraint c_vmURtoLL_geq = new GurobiConstraint();
            GurobiConstraint c_vmURtoLL_leq = new GurobiConstraint();
            GurobiConstraint c_vmLLtoUR_geq = new GurobiConstraint();
            GurobiConstraint c_vmLLtoUR_leq = new GurobiConstraint();
            if (virtualPointVars.indexOf(vp) == 0) {

                c_vmOut.setName("VM_cnn.4");
                c_vmOut.setSense('=');
                c_vmOut.addToRHS(vp.vm_detour_qs[4], 1.0);
                executor.addConstraint(c_vmOut);

                //1.VM_ul->lr.3
                c_vmULtoLR_geq.setName("VM_ul->lr.3_geq");
                c_vmULtoLR_geq.addToLHS(vp.vm_detour_qs[0], obstacles.size());
                c_vmULtoLR_geq.setSense('>');
                executor.addConstraint(c_vmULtoLR_geq);
                //todo
                c_vmULtoLR_leq.setName("VM_ul->lr.3_leq");
                c_vmULtoLR_leq.addToLHS(vp.vm_detour_qs[0], 1.0);
                c_vmULtoLR_leq.setSense('<');
                executor.addConstraint(c_vmULtoLR_leq);
                //2.VM_lr->ul.3
                c_vmLRtoUL_geq.setName("2.VM_lr->ul.3_geq");
                c_vmLRtoUL_geq.addToLHS(vp.vm_detour_qs[1], obstacles.size());
                c_vmLRtoUL_geq.setSense('>');
                executor.addConstraint(c_vmLRtoUL_geq);
                //todo
                c_vmLRtoUL_leq.setName("2.VM_lr->ul.3_leq");
                c_vmLRtoUL_leq.addToLHS(vp.vm_detour_qs[1], 1.0);
                c_vmLRtoUL_leq.setSense('<');
                executor.addConstraint(c_vmLRtoUL_leq);
                //3.VM_ur->ll.3
                c_vmURtoLL_geq.setName("3.VM_ur->ll.3_geq");
                c_vmURtoLL_geq.addToLHS(vp.vm_detour_qs[2], obstacles.size());
                c_vmURtoLL_geq.setSense('>');
                executor.addConstraint(c_vmURtoLL_geq);
                //todo
                c_vmURtoLL_leq.setName("3.VM_ur->ll.3_leq");
                c_vmURtoLL_leq.addToLHS(vp.vm_detour_qs[2], 1.0);
                c_vmURtoLL_leq.setSense('<');
                executor.addConstraint(c_vmURtoLL_leq);
                //4:VM_ll->ur.3
                c_vmLLtoUR_geq.setName("4.VM_ll->ur.3_geq");
                c_vmLLtoUR_geq.addToLHS(vp.vm_detour_qs[3], obstacles.size());
                c_vmLLtoUR_geq.setSense('>');
                executor.addConstraint(c_vmLLtoUR_geq);
                //todo
                c_vmLLtoUR_leq.setName("4.VM_ll->ur.3_leq");
                c_vmLLtoUR_leq.addToLHS(vp.vm_detour_qs[3], 1.0);
                c_vmLLtoUR_leq.setSense('<');
                executor.addConstraint(c_vmLLtoUR_leq);




                //VM_busLength
                c_busLength.addToRHS(vp.vm_dist_cq, 1.0);

                //VM_dist
                c = new GurobiConstraint();
                c.setName("VM_dist");
                c.addToLHS(vp.vm_dist_cq, 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_vmDist_iqs[2], Math.sqrt(2));
                c.addToRHS(vp.aux_vmDist_iqs[3], 1.0);
                executor.addConstraint(c);
                //auxiliary
                c = new GurobiConstraint();
                c.setName("VM_dist_auxX");
                c.addToLHS(vp.x, 1.0);
                c.setLHSConstant(-master.getX());
                c.setSense('=');
                c.addToRHS(vp.aux_vmDist_iqs[4], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_vmDist_iqs[0], vp.aux_vmDist_iqs[4], "VM_dist_absX");
                c = new GurobiConstraint();
                c.setName("VM_dist_auxY");
                c.addToLHS(vp.y, 1.0);
                c.setLHSConstant(-master.getY());
                c.setSense('=');
                c.addToRHS(vp.aux_vmDist_iqs[5], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_vmDist_iqs[1], vp.aux_vmDist_iqs[5], "VM_dist_absY");
                c = new GurobiConstraint();
                c.setName("VM_dist_auxXY");
                c.addToLHS(vp.aux_vmDist_iqs[0], 1.0);
                c.addToLHS(vp.aux_vmDist_iqs[1], -1.0);
                c.setSense('=');
                c.addToRHS(vp.aux_vmDist_iqs[6], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_vmDist_iqs[3], vp.aux_vmDist_iqs[6], "VM_dist_absXY");
                //Min_linear
                c = new GurobiConstraint();
                c.addToLHS(vp.aux_vmDist_iqs[2], 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_vmDist_iqs[0], 1.0);
                c.addToRHS(vp.auxQ_vmDist, M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.aux_vmDist_iqs[2], 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_vmDist_iqs[1], 1.0);
                c.addToRHS(vp.auxQ_vmDist, -M);
                executor.addConstraint(c);


            }


            //cnn.4
            GurobiQuadConstraint c_vpOut = new GurobiQuadConstraint();
            c_vpOut.setName("cnn.4");
            c_vpOut.setSense('=');
            c_vpOut.addToRHS(vp.detour_qs[4], 1.0);
            executor.addConstraint(c_vpOut);

            //pl
            GurobiConstraint c_dij_detour = new GurobiConstraint();
            c_dij_detour.setName("pl");
            c_dij_detour.addToLHS(vp.dist_cq, 1.0);
            c_dij_detour.setSense('>');
            c_dij_detour.addToRHS(vp.dInOut_cqs[0], 1.0);
            c_dij_detour.addToRHS(vp.dInOut_cqs[1], 1.0);
            c_dij_detour.addToRHS(vp.detour_qs[4], M);
            c_dij_detour.setRHSConstant(-M);
            executor.addConstraint(c_dij_detour);

            /*
            Obstacle relative rgd. NEXT virtualPoint
             */
            //1.ul->lr
            GurobiConstraint c_ULtoLR_geq = new GurobiConstraint();
            c_ULtoLR_geq.setName("ul->lr.3_geq");
            c_ULtoLR_geq.addToLHS(vp.detour_qs[0], obstacles.size());
            c_ULtoLR_geq.setSense('>');
            executor.addConstraint(c_ULtoLR_geq);
            GurobiConstraint c_ULtoLR_leq = new GurobiConstraint();
            c_ULtoLR_leq.setName("ul->lr.3_leq");
            c_ULtoLR_leq.addToLHS(vp.detour_qs[0], 1.0);
            c_ULtoLR_leq.setSense('<');
            executor.addConstraint(c_ULtoLR_leq);
            //2.lr->ul
            GurobiConstraint c_LRtoUL_geq = new GurobiConstraint();
            c_LRtoUL_geq.setName("lr->ul.3_geq");
            c_LRtoUL_geq.addToLHS(vp.detour_qs[1], obstacles.size());
            c_LRtoUL_geq.setSense('>');
            executor.addConstraint(c_LRtoUL_geq);
            GurobiConstraint c_LRtoUL_leq = new GurobiConstraint();
            c_LRtoUL_leq.setName("lr->ul.3_leq");
            c_LRtoUL_leq.addToLHS(vp.detour_qs[1], 1.0);
            c_LRtoUL_leq.setSense('<');
            executor.addConstraint(c_LRtoUL_leq);
            //3.ur->ll
            GurobiConstraint c_URtoLL_geq = new GurobiConstraint();
            c_URtoLL_geq.setName("ur->ll.3_geq");
            c_URtoLL_geq.addToLHS(vp.detour_qs[2], obstacles.size());
            c_URtoLL_geq.setSense('>');
            executor.addConstraint(c_URtoLL_geq);
            GurobiConstraint c_URtoLL_leq = new GurobiConstraint();
            c_URtoLL_leq.setName("ur->ll.3_leq");
            c_URtoLL_leq.addToLHS(vp.detour_qs[2], 1.0);
            c_URtoLL_leq.setSense('<');
            executor.addConstraint(c_URtoLL_leq);
            //4.ll->ur
            GurobiConstraint c_LLtoUR_geq = new GurobiConstraint();
            c_LLtoUR_geq.setName("ll->ur.3_geq");
            c_LLtoUR_geq.addToLHS(vp.detour_qs[3], obstacles.size());
            c_LLtoUR_geq.setSense('>');
            executor.addConstraint(c_LLtoUR_geq);
            GurobiConstraint c_LLtoUR_leq = new GurobiConstraint();
            c_LLtoUR_leq.setName("ll->ur.3_leq");
            c_LLtoUR_leq.addToLHS(vp.detour_qs[3], 1.0);
            c_LLtoUR_leq.setSense('<');
            executor.addConstraint(c_LLtoUR_leq);



            for (Obstacle om : obstacles) {

                c_ULtoLR_geq.addToRHS(vp.relObstacles_qs.get(om)[0], 1.0);
                c_ULtoLR_leq.addToRHS(vp.relObstacles_qs.get(om)[0], 1.0);

                c_LRtoUL_geq.addToRHS(vp.relObstacles_qs.get(om)[1], 1.0);
                c_LRtoUL_leq.addToRHS(vp.relObstacles_qs.get(om)[1], 1.0);

                c_URtoLL_geq.addToRHS(vp.relObstacles_qs.get(om)[2], 1.0);
                c_URtoLL_leq.addToRHS(vp.relObstacles_qs.get(om)[2], 1.0);

                c_LLtoUR_geq.addToRHS(vp.relObstacles_qs.get(om)[3], 1.0);
                c_LLtoUR_leq.addToRHS(vp.relObstacles_qs.get(om)[3], 1.0);

                //nonl
                c = new GurobiConstraint();
                c.setName("nonl");
                c.addToLHS(vp.non_qs.get(om)[0], 1.0);
                c.addToLHS(vp.non_qs.get(om)[1], 1.0);
                c.addToLHS(vp.non_qs.get(om)[2], 1.0);
                c.addToLHS(vp.non_qs.get(om)[3], 1.0);
                c.setSense('<');
                c.setRHSConstant(3.0);
                executor.addConstraint(c);
                //nonl.l
                c = new GurobiConstraint();
                c.setName("nonl.l_1");
                c.addToLHS(vp.x, 1.0);
                c.setSense('>');
                c.addToRHS(vp.non_qs.get(om)[0], M);
                c.setRHSConstant(om.getMinX() - eps + minDist - M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("nonl.l_2");
                c.addToLHS(vp.x, 1.0);
                c.setSense('<');
                c.addToRHS(vp.non_qs.get(om)[0], M);
                c.setRHSConstant(om.getMinX() - eps);
                executor.addConstraint(c);
                //nonl.r
                c = new GurobiConstraint();
                c.setName("nonl.r_1");
                c.addToLHS(vp.x, 1.0);
                c.setSense('<');
                c.addToRHS(vp.non_qs.get(om)[1], -M);
                c.setRHSConstant(om.getMaxX() + eps - minDist + M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("nonl.r_2");
                c.addToLHS(vp.x, 1.0);
                c.setSense('>');
                c.addToRHS(vp.non_qs.get(om)[1], -M);
                c.setRHSConstant(om.getMaxX() + eps);
                executor.addConstraint(c);
                //nonl.t
                c = new GurobiConstraint();
                c.setName("nonl.t_1");
                c.addToLHS(vp.y, 1.0);
                c.setSense('<');
                c.addToRHS(vp.non_qs.get(om)[2], -M);
                c.setRHSConstant(om.getMaxY() + eps - minDist + M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("nonl.t_2");
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.non_qs.get(om)[2], -M);
                c.setRHSConstant(om.getMaxY() + eps);
                executor.addConstraint(c);
                //nonl.b
                c = new GurobiConstraint();
                c.setName("nonl.b_1");
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.non_qs.get(om)[3], M);
                c.setRHSConstant(om.getMinY() - eps + minDist - M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("nonl.b_2");
                c.addToLHS(vp.y, 1.0);
                c.setSense('<');
                c.addToRHS(vp.non_qs.get(om)[3], M);
                c.setRHSConstant(om.getMinY() - eps);
                executor.addConstraint(c);


                //diagonal Sets
                //rel.ul
                c = new GurobiConstraint();
                c.setName("rel.ul_1");
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, 1.0);
                c.addToRHS(vp.dir_qs.get(om)[0], M);
                c.setRHSConstant(om.getMaxY() - om.getMinX() + minDist - M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("rel.ul_2");
                c.addToLHS(vp.y, 1.0);
                c.setSense('<');
                c.addToRHS(vp.x, 1.0);
                c.addToRHS(vp.dir_qs.get(om)[0], M);
                c.setRHSConstant(om.getMaxY() - om.getMinX());
                executor.addConstraint(c);

                //rel.ur
                c = new GurobiConstraint();
                c.setName("rel.ur_1");
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, -1.0);
                c.addToRHS(vp.dir_qs.get(om)[1], M);
                c.setRHSConstant(om.getMaxY() + om.getMaxX() + minDist - M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("rel.ur_2");
                c.addToLHS(vp.y, 1.0);
                c.setSense('<');
                c.addToRHS(vp.x, -1.0);
                c.addToRHS(vp.dir_qs.get(om)[1], M);
                c.setRHSConstant(om.getMaxY() + om.getMaxX());
                executor.addConstraint(c);

                //rel.lr
                c = new GurobiConstraint();
                c.setName("rel.lr_1");
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, 1.0);
                c.addToRHS(vp.dir_qs.get(om)[2], -M);
                c.setRHSConstant(om.getMinY() - om.getMaxX());
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("rel.lr_2");
                c.addToLHS(vp.y, 1.0);
                c.setSense('<');
                c.addToRHS(vp.x, 1.0);
                c.addToRHS(vp.dir_qs.get(om)[2], -M);
                c.setRHSConstant(om.getMinY() - om.getMaxX() - minDist + M);
                executor.addConstraint(c);

                //rel.ll
                c = new GurobiConstraint();
                c.setName("rel.ll_1");
                c.addToLHS(vp.y, 1.0);
                c.setSense('>');
                c.addToRHS(vp.x, -1.0);
                c.addToRHS(vp.dir_qs.get(om)[3], -M);
                c.setRHSConstant(om.getMinY() + om.getMinX());
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("rel.ll_2");
                c.addToLHS(vp.y, 1.0);
                c.setSense('<');
                c.addToRHS(vp.x, -1.0);
                c.addToRHS(vp.dir_qs.get(om)[3], -M);
                c.setRHSConstant(om.getMinY() + om.getMinX() - minDist + M);
                executor.addConstraint(c);

                //tL
                qc = new GurobiQuadConstraint();
                qc.setName("tL.1");
                qc.addToLHS(vp.dir_qs.get(om)[1], 1.0);
                qc.addToLHS(vp.dir_qs.get(om)[3], 1.0);
                qc.addToLHS(vp.non_qs.get(om)[0], vp.non_qs.get(om)[2], 1.0);
                qc.setSense('<');
                qc.addToRHS(vp.rel_qs.get(om)[0], -3.0);
                qc.setRHSConstant(3.0);
                executor.addConstraint(qc);
                /*c = new GurobiConstraint();
                c.setName("tL.linear.1");
                c.addToLHS(vp.dir_qs.get(om)[1], 1.0);
                c.addToLHS(vp.dir_qs.get(om)[3], 1.0);
                c.addToLHS(vp.rel_AusQs.get(om)[0], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(om)[0], -3.0);
                c.setRHSConstant(3.0);
                executor.addConstraint(c);*/
                qc = new GurobiQuadConstraint();
                qc.setName("tL.2");
                qc.addToLHS(vp.dir_qs.get(om)[1], 1.0);
                qc.addToLHS(vp.dir_qs.get(om)[3], 1.0);
                qc.addToLHS(vp.non_qs.get(om)[0], vp.non_qs.get(om)[2], 1.0);
                qc.setSense('>');
                qc.addToRHS(vp.rel_qs.get(om)[0], -1.0);
                qc.setRHSConstant(1.0);
                executor.addConstraint(qc);
                /*c = new GurobiConstraint();
                c.setName("tL.linear.2");
                c.addToLHS(vp.dir_qs.get(om)[1], 1.0);
                c.addToLHS(vp.dir_qs.get(om)[3], 1.0);
                c.addToLHS(vp.rel_AusQs.get(om)[0], 1.0);
                c.setSense('>');
                c.addToRHS(vp.rel_qs.get(om)[0], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);*/
                //aux
                /*c = new GurobiConstraint();
                c.setName("tL.aux_l");
                c.addToLHS(vp.rel_AusQs.get(om)[0], 1.0);
                c.setSense('<');
                c.addToRHS(vp.dir_qs.get(om)[0], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("tL.aux_t");
                c.addToLHS(vp.rel_AusQs.get(om)[0], 1.0);
                c.setSense('<');
                c.addToRHS(vp.dir_qs.get(om)[2], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("tL.aux_tl");
                c.addToLHS(vp.rel_AusQs.get(om)[0], 1.0);
                c.setSense('>');
                c.addToRHS(vp.dir_qs.get(om)[0], 1.0);
                c.addToRHS(vp.dir_qs.get(om)[2], 1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);*/





                //tR
                qc = new GurobiQuadConstraint();
                qc.setName("tR.1");
                qc.addToLHS(vp.dir_qs.get(om)[0], 1.0);
                qc.addToLHS(vp.dir_qs.get(om)[2], 1.0);
                qc.addToLHS(vp.non_qs.get(om)[1], vp.non_qs.get(om)[2], 1.0);
                qc.setSense('<');
                qc.addToRHS(vp.rel_qs.get(om)[1], -3.0);
                qc.setRHSConstant(3.0);
                executor.addConstraint(qc);
                /*c = new GurobiConstraint();
                c.setName("tR.linear.1");
                c.addToLHS(vp.dir_qs.get(om)[0], 1.0);
                c.addToLHS(vp.dir_qs.get(om)[2], 1.0);
                c.addToLHS(vp.rel_AusQs.get(om)[1], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(om)[1], -3.0);
                c.setRHSConstant(3.0);
                executor.addConstraint(c);*/
                qc = new GurobiQuadConstraint();
                qc.setName("tR.2");
                qc.addToLHS(vp.dir_qs.get(om)[0], 1.0);
                qc.addToLHS(vp.dir_qs.get(om)[2], 1.0);
                qc.addToLHS(vp.non_qs.get(om)[1], vp.non_qs.get(om)[2], 1.0);
                qc.setSense('>');
                qc.addToRHS(vp.rel_qs.get(om)[1], -1.0);
                qc.setRHSConstant(1.0);
                executor.addConstraint(qc);
                /*c = new GurobiConstraint();
                c.setName("tR.linear.2");
                c.addToLHS(vp.dir_qs.get(om)[0], 1.0);
                c.addToLHS(vp.dir_qs.get(om)[2], 1.0);
                c.addToLHS(vp.rel_AusQs.get(om)[1], 1.0);
                c.setSense('>');
                c.addToRHS(vp.rel_qs.get(om)[1], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);*/
                //aux
                /*c = new GurobiConstraint();
                c.setName("tR.aux.r");
                c.addToLHS(vp.rel_AusQs.get(om)[1], 1.0);
                c.setSense('<');
                c.addToRHS(vp.dir_qs.get(om)[1], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("tR.aux.t");
                c.addToLHS(vp.rel_AusQs.get(om)[1], 1.0);
                c.setSense('<');
                c.addToRHS(vp.dir_qs.get(om)[2], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("tR.aux.tr");
                c.addToLHS(vp.rel_AusQs.get(om)[1], 1.0);
                c.setSense('>');
                c.addToRHS(vp.dir_qs.get(om)[1], 1.0);
                c.addToRHS(vp.dir_qs.get(om)[2], 1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);*/


                //bL
                qc = new GurobiQuadConstraint();
                qc.setName("bL.1");
                qc.addToLHS(vp.dir_qs.get(om)[0], 1.0);
                qc.addToLHS(vp.dir_qs.get(om)[2], 1.0);
                qc.addToLHS(vp.non_qs.get(om)[0], vp.non_qs.get(om)[3], 1.0);
                qc.setSense('<');
                qc.addToRHS(vp.rel_qs.get(om)[2], -3.0);
                qc.setRHSConstant(3.0);
                executor.addConstraint(qc);
                /*c = new GurobiConstraint();
                c.setName("bL.linear.1");
                c.addToLHS(vp.dir_qs.get(om)[0], 1.0);
                c.addToLHS(vp.dir_qs.get(om)[2], 1.0);
                c.addToLHS(vp.rel_AusQs.get(om)[2], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(om)[2], -3.0);
                c.setRHSConstant(3.0);
                executor.addConstraint(c);*/

                qc = new GurobiQuadConstraint();
                qc.setName("bL.2");
                qc.addToLHS(vp.dir_qs.get(om)[0], 1.0);
                qc.addToLHS(vp.dir_qs.get(om)[2], 1.0);
                qc.addToLHS(vp.non_qs.get(om)[0], vp.non_qs.get(om)[3], 1.0);
                qc.setSense('>');
                qc.addToRHS(vp.rel_qs.get(om)[2], -1.0);
                qc.setRHSConstant(1.0);
                executor.addConstraint(qc);
                /*c = new GurobiConstraint();
                c.setName("bL.linear.2");
                c.addToLHS(vp.dir_qs.get(om)[0], 1.0);
                c.addToLHS(vp.dir_qs.get(om)[2], 1.0);
                c.addToLHS(vp.rel_AusQs.get(om)[2], 1.0);
                c.setSense('>');
                c.addToRHS(vp.rel_qs.get(om)[2], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);*/
                //aux
                /*c = new GurobiConstraint();
                c.setName("bL.aux.l");
                c.addToLHS(vp.rel_AusQs.get(om)[2], 1.0);
                c.setSense('<');
                c.addToRHS(vp.dir_qs.get(om)[0], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("bL.aux.b");
                c.addToLHS(vp.rel_AusQs.get(om)[2], 1.0);
                c.setSense('<');
                c.addToRHS(vp.dir_qs.get(om)[3], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("bL.aux.bl");
                c.addToLHS(vp.rel_AusQs.get(om)[2], 1.0);
                c.setSense('>');
                c.addToRHS(vp.dir_qs.get(om)[0], 1.0);
                c.addToRHS(vp.dir_qs.get(om)[3], 1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);*/

                //bR
                qc = new GurobiQuadConstraint();
                qc.setName("bR.1");
                qc.addToLHS(vp.dir_qs.get(om)[1], 1.0);
                qc.addToLHS(vp.dir_qs.get(om)[3], 1.0);
                qc.addToLHS(vp.non_qs.get(om)[1], vp.non_qs.get(om)[3], 1.0);
                qc.setSense('<');
                qc.addToRHS(vp.rel_qs.get(om)[3], -3.0);
                qc.setRHSConstant(3.0);
                executor.addConstraint(qc);
                /*c = new GurobiConstraint();
                c.setName("bR.linear.1");
                c.addToLHS(vp.dir_qs.get(om)[1], 1.0);
                c.addToLHS(vp.dir_qs.get(om)[3], 1.0);
                c.addToLHS(vp.rel_AusQs.get(om)[3], 1.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(om)[3], -3.0);
                c.setRHSConstant(3.0);
                executor.addConstraint(c);*/

                qc = new GurobiQuadConstraint();
                qc.setName("bR.2");
                qc.addToLHS(vp.dir_qs.get(om)[1], 1.0);
                qc.addToLHS(vp.dir_qs.get(om)[3], 1.0);
                qc.addToLHS(vp.non_qs.get(om)[1], vp.non_qs.get(om)[3], 1.0);
                qc.setSense('>');
                qc.addToRHS(vp.rel_qs.get(om)[3], -1.0);
                qc.setRHSConstant(1.0);
                executor.addConstraint(qc);
                /*c = new GurobiConstraint();
                c.setName("bR.linear.2");
                c.addToLHS(vp.dir_qs.get(om)[1], 1.0);
                c.addToLHS(vp.dir_qs.get(om)[3], 1.0);
                c.addToLHS(vp.rel_AusQs.get(om)[3], 1.0);
                c.setSense('>');
                c.addToRHS(vp.rel_qs.get(om)[3], -1.0);
                c.setRHSConstant(1.0);
                executor.addConstraint(c);*/
                //aux
                /*c = new GurobiConstraint();
                c.setName("bR.aux.r");
                c.addToLHS(vp.rel_AusQs.get(om)[3], 1.0);
                c.setSense('<');
                c.addToRHS(vp.dir_qs.get(om)[1], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("bR.aux.b");
                c.addToLHS(vp.rel_AusQs.get(om)[3], 1.0);
                c.setSense('<');
                c.addToRHS(vp.dir_qs.get(om)[3], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("bR.aux.br");
                c.addToLHS(vp.rel_AusQs.get(om)[3], 1.0);
                c.setSense('>');
                c.addToRHS(vp.dir_qs.get(om)[1], 1.0);
                c.addToRHS(vp.dir_qs.get(om)[3], 1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);*/

                /*
                    Connection with Master
                     */
                if (i == 0) {

                    c_vmULtoLR_geq.addToRHS(vp.vm_relObstacles_qs.get(om)[0], 1.0);
                    c_vmULtoLR_leq.addToRHS(vp.vm_relObstacles_qs.get(om)[0], 1.0);

                    c_vmLRtoUL_geq.addToRHS(vp.vm_relObstacles_qs.get(om)[1], 1.0);
                    c_vmLRtoUL_leq.addToRHS(vp.vm_relObstacles_qs.get(om)[1], 1.0);

                    c_vmURtoLL_geq.addToRHS(vp.vm_relObstacles_qs.get(om)[2], 1.0);
                    c_vmURtoLL_leq.addToRHS(vp.vm_relObstacles_qs.get(om)[2], 1.0);

                    c_vmLLtoUR_geq.addToRHS(vp.vm_relObstacles_qs.get(om)[3], 1.0);
                    c_vmLLtoUR_leq.addToRHS(vp.vm_relObstacles_qs.get(om)[3], 1.0);

                    //1.VM_ul->lr
                    c = new GurobiConstraint();
                    c.setName("1.vm_aux_ul->lr");
                    c.addToLHS(vp.vm_relObstacles_qs.get(om)[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(om)[0], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : om.get_bRObstacles()) {
                        //1.VM_ul->lr.1
                        c = new GurobiConstraint();
                        c.setName("1.VM_ul->lr.1");
                        c.addToLHS(vp.vm_relObstacles_qs.get(om)[0], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.rel_qs.get(om)[0], 1.0);
                        c.setRHSConstant(master.getPseudo_oRel_qs().get(on)[3] - 1.0);
                        executor.addConstraint(c);

                        //1.VM_ul->lr.2
                        c = new GurobiConstraint();
                        c.setName("1.VM_ul->lr.2");
                        c.addToLHS(vp.vm_relObstacles_qs.get(on)[0], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.rel_qs.get(om)[0], 1.0);
                        c.setRHSConstant(master.getPseudo_oRel_qs().get(on)[3] - 1.0);
                        executor.addConstraint(c);


                    }

                    //2.VM_lr->ul
                    c = new GurobiConstraint();
                    c.setName("2.vm_aux_lr->ul");
                    c.addToLHS(vp.vm_relObstacles_qs.get(om)[1], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(om)[3], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : om.get_tLObstacles()) {
                        //2.VM_lr->ul.1
                        c = new GurobiConstraint();
                        c.setName("2.VM_lr->ul.1");
                        c.addToLHS(vp.vm_relObstacles_qs.get(om)[1], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.rel_qs.get(om)[3], 1.0);
                        c.setRHSConstant(master.getPseudo_oRel_qs().get(on)[0] - 1.0);
                        executor.addConstraint(c);

                        //2.VM_lr->ul.2
                        c = new GurobiConstraint();
                        c.setName("2.VM_lr->ul.2");
                        c.addToLHS(vp.vm_relObstacles_qs.get(on)[1], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.rel_qs.get(om)[3], 1.0);
                        c.setRHSConstant(master.getPseudo_oRel_qs().get(on)[0] - 1.0);
                        executor.addConstraint(c);

                    }


                    //3:VM_ur->ll
                    c = new GurobiConstraint();
                    c.setName("3.vm_aux_ur->ll");
                    c.addToLHS(vp.vm_relObstacles_qs.get(om)[2], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(om)[1], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : om.get_bLObstacles()) {
                        //3:VM_ur->ll.1
                        c = new GurobiConstraint();
                        c.setName("3:VM_ur->ll.1");
                        c.addToLHS(vp.vm_relObstacles_qs.get(om)[2], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.rel_qs.get(om)[1], 1.0);
                        c.setRHSConstant(master.getPseudo_oRel_qs().get(on)[2] - 1.0);
                        executor.addConstraint(c);

                        //3:VM_ur->ll.2
                        c = new GurobiConstraint();
                        c.setName("3:VM_ur->ll.2");
                        c.addToLHS(vp.vm_relObstacles_qs.get(on)[2], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.rel_qs.get(om)[1], 1.0);
                        c.setRHSConstant(master.getPseudo_oRel_qs().get(on)[2] - 1.0);
                        executor.addConstraint(c);


                    }


                    //4:VM_ll->ur
                    c = new GurobiConstraint();
                    c.setName("4.vm_aux_ll->ur");
                    c.addToLHS(vp.vm_relObstacles_qs.get(om)[3], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(om)[2], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : om.get_tRObstacles()) {
                        //4:VM_ll->ur.1
                        c = new GurobiConstraint();
                        c.setName("4:VM_ll->ur.1");
                        c.addToLHS(vp.vm_relObstacles_qs.get(om)[3], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.rel_qs.get(om)[2], 1.0);
                        c.setRHSConstant(master.getPseudo_oRel_qs().get(on)[1] - 1.0);
                        executor.addConstraint(c);


                        //4:VM_ll->ur.2
                        c = new GurobiConstraint();
                        c.setName("4:VM_ll->ur.2");
                        c.addToLHS(vp.vm_relObstacles_qs.get(on)[3], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.rel_qs.get(om)[2], 1.0);
                        c.setRHSConstant(master.getPseudo_oRel_qs().get(on)[1] - 1.0);
                        executor.addConstraint(c);
                    }


                    //VM_aux.1
                    c = new GurobiConstraint();
                    c.setName("VM_aux1.1");
                    c.addToLHS(vp.vm_relObstacles_qs.get(om)[0], 1.0);
                    c.addToLHS(vp.vm_relObstacles_qs.get(om)[1], 1.0);
                    c.addToLHS(vp.vm_relObstacles_qs.get(om)[2], 1.0);
                    c.addToLHS(vp.vm_relObstacles_qs.get(om)[3], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.vm_relObstacles_qs.get(om)[4], 4.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("VM_aux1.2");
                    c.addToLHS(vp.vm_relObstacles_qs.get(om)[0], 1.0);
                    c.addToLHS(vp.vm_relObstacles_qs.get(om)[1], 1.0);
                    c.addToLHS(vp.vm_relObstacles_qs.get(om)[2], 1.0);
                    c.addToLHS(vp.vm_relObstacles_qs.get(om)[3], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.vm_relObstacles_qs.get(om)[4], 1.0);
                    executor.addConstraint(c);


                    //VM_aux.2
                    c = new GurobiConstraint();
                    c.setName("VM_aux.2.1");
                    c.addToLHS(vp.vm_detour_qs[0], 1.0);
                    c.addToLHS(vp.vm_detour_qs[1], 1.0);
                    c.addToLHS(vp.vm_detour_qs[2], 1.0);
                    c.addToLHS(vp.vm_detour_qs[3], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.vm_detour_qs[4], 4.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("VM_aux.2.2");
                    c.addToLHS(vp.vm_detour_qs[0], 1.0);
                    c.addToLHS(vp.vm_detour_qs[1], 1.0);
                    c.addToLHS(vp.vm_detour_qs[2], 1.0);
                    c.addToLHS(vp.vm_detour_qs[3], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.vm_detour_qs[4], 1.0);
                    executor.addConstraint(c);


                    //VM_cnn_1
                    c = new GurobiConstraint();
                    c.setName("VM_cnn_1");
                    c.addToLHS(vp.vm_corner_qs.get(om)[0], 1.0);
                    c.addToLHS(vp.vm_corner_qs.get(om)[1], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.vm_relObstacles_qs.get(om)[0], 1.0);
                    c.addToRHS(vp.vm_relObstacles_qs.get(om)[1], 1.0);
                    executor.addConstraint(c);
                    //VM_cnn_2
                    c = new GurobiConstraint();
                    c.setName("VM_cnn_2");
                    c.addToLHS(vp.vm_corner_qs.get(om)[2], 1.0);
                    c.addToLHS(vp.vm_corner_qs.get(om)[3], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.vm_relObstacles_qs.get(om)[2], 1.0);
                    c.addToRHS(vp.vm_relObstacles_qs.get(om)[3], 1.0);
                    executor.addConstraint(c);

                    //VM_cnn.1
                    c = new GurobiConstraint();
                    c.setName("VM_cnn.1");
                    c.addToLHS(vp.vm_omOnCnn_q.get(om).get(om), 1.0);
                    c.setSense('=');
                    c.setRHSConstant(0.0);
                    executor.addConstraint(c);

                    //VM_cnn.4:LHS
                    c_vmOut.addToLHS(vp.vm_inOutCnn_qs.get(om)[0], vp.vm_relObstacles_qs.get(om)[4], 1.0);

                    //VM_cor.x
                    c = new GurobiConstraint();
                    c.setName("VM_cor.x_leq");
                    c.addToLHS(vp.vm_oCoordinate_iqs.get(om)[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.vm_corner_qs.get(om)[0], om.getMinX());
                    c.addToRHS(vp.vm_corner_qs.get(om)[2], om.getMinX());
                    c.addToRHS(vp.vm_corner_qs.get(om)[1], om.getMaxX());
                    c.addToRHS(vp.vm_corner_qs.get(om)[3], om.getMaxX());
                    c.addToRHS(vp.vm_relObstacles_qs.get(om)[4], -M);
                    c.setRHSConstant(M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("VM_cor.x_geq");
                    c.addToLHS(vp.vm_oCoordinate_iqs.get(om)[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.vm_corner_qs.get(om)[0], om.getMinX());
                    c.addToRHS(vp.vm_corner_qs.get(om)[2], om.getMinX());
                    c.addToRHS(vp.vm_corner_qs.get(om)[1], om.getMaxX());
                    c.addToRHS(vp.vm_corner_qs.get(om)[3], om.getMaxX());
                    c.addToRHS(vp.vm_relObstacles_qs.get(om)[4], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);

                    //VM_cor.y
                    c = new GurobiConstraint();
                    c.setName("VM_cor.y_leq");
                    c.addToLHS(vp.vm_oCoordinate_iqs.get(om)[1], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.vm_corner_qs.get(om)[0], om.getMinY());
                    c.addToRHS(vp.vm_corner_qs.get(om)[3], om.getMinY());
                    c.addToRHS(vp.vm_corner_qs.get(om)[1], om.getMaxY());
                    c.addToRHS(vp.vm_corner_qs.get(om)[2], om.getMaxY());
                    c.addToRHS(vp.vm_relObstacles_qs.get(om)[4], -M);
                    c.setRHSConstant(M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("VM_cor.y_geq");
                    c.addToLHS(vp.vm_oCoordinate_iqs.get(om)[1], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.vm_corner_qs.get(om)[0], om.getMinY());
                    c.addToRHS(vp.vm_corner_qs.get(om)[3], om.getMinY());
                    c.addToRHS(vp.vm_corner_qs.get(om)[1], om.getMaxY());
                    c.addToRHS(vp.vm_corner_qs.get(om)[2], om.getMaxY());
                    c.addToRHS(vp.vm_relObstacles_qs.get(om)[4], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);

                    //VM_d_i_ms-> (pl.1)
                    c = new GurobiConstraint();
                    c.setName("VM_d_i_ms->");
                    c.addToLHS(vp.vm_dInOut_cqs[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_vmdOut_iqs.get(om)[2], Math.sqrt(2));
                    c.addToRHS(vp.aux_vmdOut_iqs.get(om)[3], 1.0);
                    c.addToRHS(vp.vm_inOutCnn_qs.get(om)[0], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    //auxiliary
                    c = new GurobiConstraint();
                    c.setName("VM_d_i_ms->_auxX");
                    c.addToLHS(vp.x, 1.0);
                    c.addToLHS(vp.vm_oCoordinate_iqs.get(om)[0], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_vmdOut_iqs.get(om)[4], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_vmdOut_iqs.get(om)[0], vp.aux_vmdOut_iqs.get(om)[4], om.getName() + "VM_d_i_ms->_absX");
                    c = new GurobiConstraint();
                    c.setName("VM_d_i_ms->_auxY");
                    c.addToLHS(vp.y, 1.0);
                    c.addToLHS(vp.vm_oCoordinate_iqs.get(om)[1], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_vmdOut_iqs.get(om)[5], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_vmdOut_iqs.get(om)[1], vp.aux_vmdOut_iqs.get(om)[5], om.getName() + "VM_d_i_ms->_absY");
                    c = new GurobiConstraint();
                    c.setName("VM_d_i_ms->_auxXY");
                    c.addToLHS(vp.aux_vmdOut_iqs.get(om)[0], 1.0);
                    c.addToLHS(vp.aux_vmdOut_iqs.get(om)[1], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_vmdOut_iqs.get(om)[6], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_vmdOut_iqs.get(om)[3], vp.aux_vmdOut_iqs.get(om)[6], om.getName() + "VM_d_i_ms->_absXY");
                    //Min_2
                    c = new GurobiConstraint();
                    c.addToLHS(vp.aux_vmdOut_iqs.get(om)[2], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_vmdOut_iqs.get(om)[0], 1.0);
                    c.addToRHS(vp.auxQ_vmdOut.get(om), M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(vp.aux_vmdOut_iqs.get(om)[2], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_vmdOut_iqs.get(om)[1], 1.0);
                    c.addToRHS(vp.auxQ_vmdOut.get(om), -M);
                    executor.addConstraint(c);

                    //VM_d_i_ms<- (pl.3)
                    c = new GurobiConstraint();
                    c.addToLHS(vp.vm_dInOut_cqs[1], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_vmdIn_iqs.get(om)[2], Math.sqrt(2));
                    c.addToRHS(vp.aux_vmdIn_iqs.get(om)[3], 1.0);
                    c.addToRHS(vp.vm_inOutCnn_qs.get(om)[1], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    //auxiliary
                    c = new GurobiConstraint();
                    c.setName("VM_d_i_ms<-_auxX");
                    c.addToLHS(vp.vm_oCoordinate_iqs.get(om)[0], 1.0);
                    c.setLHSConstant(-master.getX());
                    c.setSense('=');
                    c.addToRHS(vp.aux_vmdIn_iqs.get(om)[4], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_vmdIn_iqs.get(om)[0], vp.aux_vmdIn_iqs.get(om)[4], om.getName() + "VM_d_i_ms<-_absX");
                    c = new GurobiConstraint();
                    c.setName("VM_d_i_ms<-_auxY");
                    c.addToLHS(vp.vm_oCoordinate_iqs.get(om)[1], 1.0);
                    c.setLHSConstant(-master.getY());
                    c.setSense('=');
                    c.addToRHS(vp.aux_vmdIn_iqs.get(om)[5], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_vmdIn_iqs.get(om)[1], vp.aux_vmdIn_iqs.get(om)[5], om.getName() + "VM_d_i_ms<-_absY");
                    c = new GurobiConstraint();
                    c.setName("VM_d_i_ms<-_auxXY");
                    c.addToLHS(vp.aux_vmdIn_iqs.get(om)[0], 1.0);
                    c.addToLHS(vp.aux_vmdIn_iqs.get(om)[1], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_vmdIn_iqs.get(om)[6], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_vmdIn_iqs.get(om)[3], vp.aux_vmdIn_iqs.get(om)[6], om.getName() + "VM_d_i_ms<-absXY");
                    //Min_linear
                    c = new GurobiConstraint();
                    c.addToLHS(vp.aux_vmdIn_iqs.get(om)[2], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_vmdIn_iqs.get(om)[0], 1.0);
                    c.addToRHS(vp.auxQ_vmdIn.get(om), M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(vp.aux_vmdIn_iqs.get(om)[2], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_vmdIn_iqs.get(om)[1], 1.0);
                    c.addToRHS(vp.auxQ_vmdIn.get(om), -M);
                    executor.addConstraint(c);


                }
                /*
                Connection with next virtualPoint
                 */

                if (i < virtualPointVars.size() - 1) {
                    vpN = virtualPointVars.get(virtualPointVars.indexOf(vp) + 1);


                    //(1)ul->lr
                    //Auxiliary
                    c = new GurobiConstraint();
                    c.setName("vv_aux_ul->lr");
                    c.addToLHS(vp.relObstacles_qs.get(om)[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(om)[0], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : om.get_bRObstacles()) {
                        //ul->lr.1
                        c = new GurobiConstraint();
                        c.setName("ul->lr.1");
                        c.addToLHS(vp.relObstacles_qs.get(om)[0], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.rel_qs.get(on)[3], 1.0);
                        c.addToRHS(vp.rel_qs.get(om)[0], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);

                        //(ul->lr.2)
                        if (!om.getName().equals(on.getName())) {
                            c = new GurobiConstraint();
                            c.setName("ul->lr.2");
                            c.addToLHS(vp.relObstacles_qs.get(on)[0], 1.0);
                            c.setSense('>');
                            c.addToRHS(vpN.rel_qs.get(on)[3], 1.0);
                            c.addToRHS(vp.rel_qs.get(om)[0], 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);
                        }
                    }

                    //(2)lr->ul
                    c = new GurobiConstraint();
                    c.setName("vv_aux_lr->ul");
                    c.addToLHS(vp.relObstacles_qs.get(om)[1], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(om)[3], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : om.get_tLObstacles()) {
                        //lr->ul.1
                        c = new GurobiConstraint();
                        c.setName("lr->ul.1");
                        c.addToLHS(vp.relObstacles_qs.get(om)[1], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.rel_qs.get(on)[0], 1.0);
                        c.addToRHS(vp.rel_qs.get(om)[3], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);

                        if (!om.getName().equals(on.getName())) {
                            //lr->ul.2
                            c = new GurobiConstraint();
                            c.setName("lr->ul.2");
                            c.addToLHS(vp.relObstacles_qs.get(on)[1], 1.0);
                            c.setSense('>');
                            c.addToRHS(vpN.rel_qs.get(on)[0], 1.0);
                            c.addToRHS(vp.rel_qs.get(om)[3], 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);
                        }
                    }

                    //ur->ll
                    c = new GurobiConstraint();
                    c.setName("vv_aux_ur->ll");
                    c.addToLHS(vp.relObstacles_qs.get(om)[2], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(om)[1], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : om.get_bLObstacles()) {
                        //ur->ll.1
                        c = new GurobiConstraint();
                        c.setName("ur->ll.1");
                        c.addToLHS(vp.relObstacles_qs.get(om)[2], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.rel_qs.get(on)[2], 1.0);
                        c.addToRHS(vp.rel_qs.get(om)[1], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);

                        //ur->ll.2
                        if (!om.getName().equals(on.getName())) {
                            c = new GurobiConstraint();
                            c.setName("ur->ll.2");
                            c.addToLHS(vp.relObstacles_qs.get(on)[2], 1.0);
                            c.setSense('>');
                            c.addToRHS(vpN.rel_qs.get(on)[2], 1.0);
                            c.addToRHS(vp.rel_qs.get(om)[1], 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);
                        }
                    }

                    //ll->ur
                    c = new GurobiConstraint();
                    c.setName("vv_aux_ll->ur");
                    c.addToLHS(vp.relObstacles_qs.get(om)[3], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(om)[2], 1.0);
                    executor.addConstraint(c);
                    for (Obstacle on : om.get_tRObstacles()) {
                        //ll->ur.1
                        c = new GurobiConstraint();
                        c.setName("ll->ur.1");
                        c.addToLHS(vp.relObstacles_qs.get(om)[3], 1.0);
                        c.setSense('>');
                        c.addToRHS(vpN.rel_qs.get(on)[1], 1.0);
                        c.addToRHS(vp.rel_qs.get(om)[2], 1.0);
                        c.setRHSConstant(-1.0);
                        executor.addConstraint(c);

                        if (!om.getName().equals(on.getName())) {
                            //ll->ur.2
                            c = new GurobiConstraint();
                            c.setName("ll->ur.2");
                            c.addToLHS(vp.relObstacles_qs.get(on)[3], 1.0);
                            c.setSense('>');
                            c.addToRHS(vpN.rel_qs.get(on)[1], 1.0);
                            c.addToRHS(vp.rel_qs.get(om)[2], 1.0);
                            c.setRHSConstant(-1.0);
                            executor.addConstraint(c);
                        }
                    }

                    c = new GurobiConstraint();
                    c.setName("aux.1.1");
                    c.addToLHS(vp.relObstacles_qs.get(om)[0], 1.0);
                    c.addToLHS(vp.relObstacles_qs.get(om)[1], 1.0);
                    c.addToLHS(vp.relObstacles_qs.get(om)[2], 1.0);
                    c.addToLHS(vp.relObstacles_qs.get(om)[3], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.relObstacles_qs.get(om)[4], 4.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("aux.1.2");
                    c.addToLHS(vp.relObstacles_qs.get(om)[0], 1.0);
                    c.addToLHS(vp.relObstacles_qs.get(om)[1], 1.0);
                    c.addToLHS(vp.relObstacles_qs.get(om)[2], 1.0);
                    c.addToLHS(vp.relObstacles_qs.get(om)[3], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.relObstacles_qs.get(om)[4], 1.0);
                    executor.addConstraint(c);


                    //aux.2
                    c = new GurobiConstraint();
                    c.setName("aux.2.1");
                    c.addToLHS(vp.detour_qs[0], 1.0);
                    c.addToLHS(vp.detour_qs[1], 1.0);
                    c.addToLHS(vp.detour_qs[2], 1.0);
                    c.addToLHS(vp.detour_qs[3], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.detour_qs[4], 4.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("aux.2.2");
                    c.addToLHS(vp.detour_qs[0], 1.0);
                    c.addToLHS(vp.detour_qs[1], 1.0);
                    c.addToLHS(vp.detour_qs[2], 1.0);
                    c.addToLHS(vp.detour_qs[3], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.detour_qs[4], 1.0);
                    executor.addConstraint(c);


                    /*
                    Connection Rules:
                     */
                    //Corner
                    c = new GurobiConstraint();
                    c.setName("corner.1");
                    c.addToLHS(vp.corner_qs.get(om)[0], 1.0);
                    c.addToLHS(vp.corner_qs.get(om)[1], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.relObstacles_qs.get(om)[0], 1.0);
                    c.addToRHS(vp.relObstacles_qs.get(om)[1], 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("corner.2");
                    c.addToLHS(vp.corner_qs.get(om)[2], 1.0);
                    c.addToLHS(vp.corner_qs.get(om)[3], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.relObstacles_qs.get(om)[2], 1.0);
                    c.addToRHS(vp.relObstacles_qs.get(om)[3], 1.0);
                    executor.addConstraint(c);

                    //cnn.1
                    c = new GurobiConstraint();
                    c.setName("cnn.1");
                    c.addToLHS(vp.omOnCnn_q.get(om).get(om), 1.0);
                    c.setSense('=');
                    c.setRHSConstant(0.0);
                    executor.addConstraint(c);

                    //cnn.3_1
                    GurobiQuadConstraint c_outCnn = new GurobiQuadConstraint();
                    c_outCnn.setName("cnn.3_1");
                    c_outCnn.addToLHS(vp.relObstacles_qs.get(om)[4], 1.0);
                    c_outCnn.setSense('=');
                    c_outCnn.addToRHS(vp.inOutCnn_qs.get(om)[1], 1.0);
                    executor.addConstraint(c_outCnn);


                    //VM_cnn.3_1
                    GurobiQuadConstraint c_vmOutCnn = new GurobiQuadConstraint();
                    //VM_cnn.3_2
                    GurobiQuadConstraint c_vmInCnn = new GurobiQuadConstraint();
                    //VM_cnn.5_1
                    GurobiConstraint c_vmCnnAux1 = new GurobiConstraint();
                    //VM_cnn.5_2
                    GurobiConstraint c_vmCnnAux2 = new GurobiConstraint();
                    if (virtualPointVars.indexOf(vp) == 0) {
                        c_vmOutCnn.setName("VM_cnn.3_1");
                        c_vmOutCnn.addToLHS(vp.vm_relObstacles_qs.get(om)[4], 1.0);
                        c_vmOutCnn.setSense('=');
                        c_vmOutCnn.addToRHS(vp.vm_inOutCnn_qs.get(om)[1], 1.0);
                        executor.addConstraint(c_vmOutCnn);

                        c_vmInCnn.setName("VM_cnn.3_2");
                        c_vmInCnn.addToLHS(vp.vm_relObstacles_qs.get(om)[4], 1.0);
                        c_vmInCnn.setSense('=');
                        c_vmInCnn.addToRHS(vp.vm_inOutCnn_qs.get(om)[1], 1.0);
                        executor.addConstraint(c_vmInCnn);

                        c_vmCnnAux1.setName("VM_cnn.5_1");
                        c_vmCnnAux1.addToLHS(vp.vm_relObstacles_qs.get(om)[4], 1.0);
                        c_vmCnnAux1.setSense('>');
                        executor.addConstraint(c_vmCnnAux1);

                        c_vmCnnAux2.setName("VM_cnn.5_2");
                        c_vmCnnAux2.addToLHS(vp.vm_relObstacles_qs.get(om)[4], 1.0);
                        c_vmCnnAux2.setSense('>');
                        executor.addConstraint(c_vmCnnAux2);
                    }

                    //cnn.3_2
                    GurobiQuadConstraint c_inCnn = new GurobiQuadConstraint();
                    c_inCnn.setName("cnn.3_2");
                    c_inCnn.addToLHS(vp.relObstacles_qs.get(om)[4], 1.0);
                    c_inCnn.setSense('=');
                    c_inCnn.addToRHS(vp.inOutCnn_qs.get(om)[0], 1.0);
                    executor.addConstraint(c_inCnn);
                    //cnn.5_1
                    GurobiConstraint c_CnnAux1 = new GurobiConstraint();
                    c_CnnAux1.setName("cnn.5_1");
                    c_CnnAux1.addToLHS(vp.relObstacles_qs.get(om)[4], 1.0);
                    c_CnnAux1.setSense('>');
                    executor.addConstraint(c_CnnAux1);
                    //cnn.5_2
                    GurobiConstraint c_CnnAux2 = new GurobiConstraint();
                    c_CnnAux2.setName("cnn.5_2");
                    c_CnnAux2.addToLHS(vp.relObstacles_qs.get(om)[4], 1.0);
                    c_CnnAux2.setSense('>');
                    executor.addConstraint(c_CnnAux2);




                    //Om ----> On
                    for (Obstacle on : obstacles) {
                        if (!om.getName().equals(on.getName())) {

                            c_CnnAux1.addToRHS(vp.omOnCnn_q.get(om).get(on), 1.0);
                            c_CnnAux2.addToRHS(vp.omOnCnn_q.get(on).get(om), 1.0);

                            /*
                            Connection with Master
                             */
                            if (virtualPointVars.indexOf(vp) == 0) {
                                //VM_cnn.2
                                c = new GurobiConstraint();
                                c.setName("VM_cnn.2");
                                c.addToLHS(vp.vm_omOnCnn_q.get(om).get(on), 1.0);
                                c.addToLHS(vp.vm_omOnCnn_q.get(on).get(om), 1.0);
                                c.setSense('<');
                                c.setRHSConstant(1.0);
                                executor.addConstraint(c);

                                //VM_cnn.3_1
                                c_vmOutCnn.addToRHS(vp.vm_omOnCnn_q.get(om).get(on), vp.vm_relObstacles_qs.get(on)[4], 1.0);
                                //VM_cnn.3_2
                                c_vmInCnn.addToRHS(vp.vm_omOnCnn_q.get(on).get(om), vp.vm_relObstacles_qs.get(om)[4], 1.0);

                                //VM_cnn.5_1
                                c_vmCnnAux1.addToRHS(vp.vm_omOnCnn_q.get(om).get(on), 1.0);
                                //VM_cnn.5_2
                                c_vmCnnAux2.addToRHS(vp.vm_omOnCnn_q.get(on).get(om), 1.0);


                                //VM_d_i_msdOmOn (pl.2)
                                c = new GurobiConstraint();
                                c.setName("VM_d_i_msdOmOn");
                                c.addToLHS(vp.vm_dOmOn_cq.get(om).get(on), 1.0);
                                c.setSense('>');
                                c.addToRHS(vp.aux_vmdOmOn_iqs.get(om).get(on)[2], Math.sqrt(2));
                                c.addToRHS(vp.aux_vmdOmOn_iqs.get(om).get(on)[3], 1.0);
                                c.addToRHS(vp.vm_omOnCnn_q.get(om).get(on), M);
                                c.setRHSConstant(-M);
                                executor.addConstraint(c);
                                //auxiliary
                                c = new GurobiConstraint();
                                c.setName("VM_d_i_msdOmOn_auxX");
                                c.addToLHS(vp.vm_oCoordinate_iqs.get(om)[0], 1.0);
                                c.addToLHS(vp.vm_oCoordinate_iqs.get(on)[0], -1.0);
                                c.setSense('=');
                                c.addToRHS(vp.aux_vmdOmOn_iqs.get(om).get(on)[4], 1.0);
                                executor.addConstraint(c);
                                executor.addGenConstraintAbs(vp.aux_vmdOmOn_iqs.get(om).get(on)[0], vp.aux_vmdOmOn_iqs.get(om).get(on)[4], om.getName() + "->" + on.getName() + "VM_d_i_msdOmOn_absX");
                                c = new GurobiConstraint();
                                c.setName("VM_d_i_msdOmOn_auxY");
                                c.addToLHS(vp.vm_oCoordinate_iqs.get(om)[1], 1.0);
                                c.addToLHS(vp.vm_oCoordinate_iqs.get(on)[1], -1.0);
                                c.setSense('=');
                                c.addToRHS(vp.aux_vmdOmOn_iqs.get(om).get(on)[5], 1.0);
                                executor.addConstraint(c);
                                executor.addGenConstraintAbs(vp.aux_vmdOmOn_iqs.get(om).get(on)[1], vp.aux_vmdOmOn_iqs.get(om).get(on)[5], om.getName() + "->" + on.getName() + "VM_d_i_msdOmOn_absY");
                                c = new GurobiConstraint();
                                c.setName("VM_d_i_msdOmOn_auxXY");
                                c.addToLHS(vp.aux_vmdOmOn_iqs.get(om).get(on)[0], 1.0);
                                c.addToLHS(vp.aux_vmdOmOn_iqs.get(om).get(on)[1], -1.0);
                                c.setSense('=');
                                c.addToRHS(vp.aux_vmdOmOn_iqs.get(om).get(on)[6], 1.0);
                                executor.addConstraint(c);
                                executor.addGenConstraintAbs(vp.aux_vmdOmOn_iqs.get(om).get(on)[3], vp.aux_vmdOmOn_iqs.get(om).get(on)[6], om.getName() + "->" + on.getName() + "VM_d_i_msdOmOn_absXY");
                                //Min_linear
                                c = new GurobiConstraint();
                                c.addToLHS(vp.aux_vmdOmOn_iqs.get(om).get(on)[2], 1.0);
                                c.setSense('>');
                                c.addToRHS(vp.aux_vmdOmOn_iqs.get(om).get(on)[0], 1.0);
                                c.addToRHS(vp.auxQ_vmdOmOn.get(om).get(on), M);
                                c.setRHSConstant(-M);
                                executor.addConstraint(c);
                                c = new GurobiConstraint();
                                c.addToLHS(vp.aux_vmdOmOn_iqs.get(om).get(on)[2], 1.0);
                                c.setSense('>');
                                c.addToRHS(vp.aux_vmdOmOn_iqs.get(om).get(on)[1], 1.0);
                                c.addToRHS(vp.auxQ_vmdOmOn.get(om).get(on), -M);
                                executor.addConstraint(c);


                            }


                            //cnn.2
                            c = new GurobiConstraint();
                            c.setName("cnn.2");
                            c.addToLHS(vp.omOnCnn_q.get(om).get(on), 1.0);
                            c.addToLHS(vp.omOnCnn_q.get(on).get(om), 1.0);
                            c.setSense('<');
                            c.setRHSConstant(1.0);
                            executor.addConstraint(c);

                            //cnn.3_1
                            c_outCnn.addToRHS(vp.omOnCnn_q.get(om).get(on), vp.relObstacles_qs.get(on)[4], 1.0);
                            //cnn.3_2
                            c_inCnn.addToRHS(vp.omOnCnn_q.get(on).get(om), vp.relObstacles_qs.get(om)[4], 1.0);

                            //d_ij: m->n (pl.2)
                            c = new GurobiConstraint();
                            c.setName("d_ij: m->n (pl.2)");
                            c.addToLHS(vp.dOmOn_cq.get(om).get(on), 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.aux_dOmOn_iqs.get(om).get(on)[2], Math.sqrt(2));
                            c.addToRHS(vp.aux_dOmOn_iqs.get(om).get(on)[3], 1.0);
                            c.addToRHS(vp.omOnCnn_q.get(om).get(on), M);
                            c.setRHSConstant(-M);
                            executor.addConstraint(c);
                            //auxiliary
                            c = new GurobiConstraint();
                            c.setName("d_ij: m->n:aux_X");
                            c.addToLHS(vp.oCoordinate_iqs.get(om)[0], 1.0);
                            c.addToLHS(vp.oCoordinate_iqs.get(on)[0], -1.0);
                            c.setSense('=');
                            c.addToRHS(vp.aux_dOmOn_iqs.get(om).get(on)[4], 1.0);
                            executor.addConstraint(c);
                            executor.addGenConstraintAbs(vp.aux_dOmOn_iqs.get(om).get(on)[0], vp.aux_dOmOn_iqs.get(om).get(on)[4], "v" + virtualPointVars.indexOf(vp) + om.getName() + "->" + on.getName() + "absX_dOmOn");
                            c = new GurobiConstraint();
                            c.setName("d_ij: m->n:aux_Y");
                            c.addToLHS(vp.oCoordinate_iqs.get(om)[1], 1.0);
                            c.addToLHS(vp.oCoordinate_iqs.get(on)[1], -1.0);
                            c.setSense('=');
                            c.addToRHS(vp.aux_dOmOn_iqs.get(om).get(on)[5], 1.0);
                            executor.addConstraint(c);
                            executor.addGenConstraintAbs(vp.aux_dOmOn_iqs.get(om).get(on)[1], vp.aux_dOmOn_iqs.get(om).get(on)[5], "v" + virtualPointVars.indexOf(vp) + om.getName() + "->" + on.getName() + "absY_dOmOn");
                            c = new GurobiConstraint();
                            c.setName("d_ij: m->n:aux_XY");
                            c.addToLHS(vp.aux_dOmOn_iqs.get(om).get(on)[0], 1.0);
                            c.addToLHS(vp.aux_dOmOn_iqs.get(om).get(on)[1], -1.0);
                            c.setSense('=');
                            c.addToRHS(vp.aux_dOmOn_iqs.get(om).get(on)[6], 1.0);
                            executor.addConstraint(c);
                            executor.addGenConstraintAbs(vp.aux_dOmOn_iqs.get(om).get(on)[3], vp.aux_dOmOn_iqs.get(om).get(on)[6], "v" + virtualPointVars.indexOf(vp) + om.getName() + "->" + on.getName() + "absXY_dOmOn");
                            //Min_linear
                            c = new GurobiConstraint();
                            c.addToLHS(vp.aux_dOmOn_iqs.get(om).get(on)[2], 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.aux_dOmOn_iqs.get(om).get(on)[0], 1.0);
                            c.addToRHS(vp.auxQ_dOmOn.get(om).get(on), M);
                            c.setRHSConstant(-M);
                            executor.addConstraint(c);
                            c = new GurobiConstraint();
                            c.addToLHS(vp.aux_dOmOn_iqs.get(om).get(on)[2], 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.aux_dOmOn_iqs.get(om).get(on)[1], 1.0);
                            c.addToRHS(vp.auxQ_dOmOn.get(om).get(on), -M);
                            executor.addConstraint(c);


                            //pl
                            c_dij_detour.addToRHS(vp.dOmOn_cq.get(om).get(on), 1.0);


                        }
                    }

                    //cnn.4 LHS
                    c_vpOut.addToLHS(vp.inOutCnn_qs.get(om)[0], vp.relObstacles_qs.get(om)[4], 1.0);

                    //cor.x
                    c = new GurobiConstraint();
                    c.setName("cor.x_leq");
                    c.addToLHS(vp.oCoordinate_iqs.get(om)[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.corner_qs.get(om)[0], om.getMinX());
                    c.addToRHS(vp.corner_qs.get(om)[2], om.getMinX());
                    c.addToRHS(vp.corner_qs.get(om)[1], om.getMaxX());
                    c.addToRHS(vp.corner_qs.get(om)[3], om.getMaxX());
                    c.addToRHS(vp.relObstacles_qs.get(om)[4], -M);
                    c.setRHSConstant(M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("cor.x_geq");
                    c.addToLHS(vp.oCoordinate_iqs.get(om)[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.corner_qs.get(om)[0], om.getMinX());
                    c.addToRHS(vp.corner_qs.get(om)[2], om.getMinX());
                    c.addToRHS(vp.corner_qs.get(om)[1], om.getMaxX());
                    c.addToRHS(vp.corner_qs.get(om)[3], om.getMaxX());
                    c.addToRHS(vp.relObstacles_qs.get(om)[4], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);

                    //cor.y
                    c = new GurobiConstraint();
                    c.setName("cor.y_leq");
                    c.addToLHS(vp.oCoordinate_iqs.get(om)[1], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.corner_qs.get(om)[0], om.getMinY());
                    c.addToRHS(vp.corner_qs.get(om)[3], om.getMinY());
                    c.addToRHS(vp.corner_qs.get(om)[1], om.getMaxY());
                    c.addToRHS(vp.corner_qs.get(om)[2], om.getMaxY());
                    c.addToRHS(vp.relObstacles_qs.get(om)[4], -M);
                    c.setRHSConstant(M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("cor.y_geq");
                    c.addToLHS(vp.oCoordinate_iqs.get(om)[1], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.corner_qs.get(om)[0], om.getMinY());
                    c.addToRHS(vp.corner_qs.get(om)[3], om.getMinY());
                    c.addToRHS(vp.corner_qs.get(om)[1], om.getMaxY());
                    c.addToRHS(vp.corner_qs.get(om)[2], om.getMaxY());
                    c.addToRHS(vp.relObstacles_qs.get(om)[4], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);

                    //d_ij:-> (pl.1)
                    c = new GurobiConstraint();
                    c.setName("d_ij:-> (pl.1)");
                    c.addToLHS(vp.dInOut_cqs[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dOut_iqs.get(om)[2], Math.sqrt(2));
                    c.addToRHS(vp.aux_dOut_iqs.get(om)[3], 1.0);
                    c.addToRHS(vp.inOutCnn_qs.get(om)[0], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    //aux
                    c = new GurobiConstraint();
                    c.setName("d_ij:->:aux_X");
                    c.addToLHS(vp.aux_dOut_iqs.get(om)[4], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.x, 1.0);
                    c.addToRHS(vp.oCoordinate_iqs.get(om)[0], -1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_dOut_iqs.get(om)[0], vp.aux_dOut_iqs.get(om)[4], "v" + virtualPointVars.indexOf(vp) + om.getName() + "abs_x_Out");
                    c = new GurobiConstraint();
                    c.setName("d_ij:->:aux_Y");
                    c.addToLHS(vp.aux_dOut_iqs.get(om)[5], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.y, 1.0);
                    c.addToRHS(vp.oCoordinate_iqs.get(om)[1], -1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_dOut_iqs.get(om)[1], vp.aux_dOut_iqs.get(om)[5], "v" + virtualPointVars.indexOf(vp) + om.getName() + "abs_y_Out");
                    c = new GurobiConstraint();
                    c.setName("d_ij:->:aux_XY");
                    c.addToLHS(vp.aux_dOut_iqs.get(om)[6], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_dOut_iqs.get(om)[0], 1.0);
                    c.addToRHS(vp.aux_dOut_iqs.get(om)[1], -1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_dOut_iqs.get(om)[3], vp.aux_dOut_iqs.get(om)[6], "v" + virtualPointVars.indexOf(vp) + om.getName() + "abs(abs_x-abs_y)");
                    //Min_linear
                    c = new GurobiConstraint();
                    c.addToLHS(vp.aux_dOut_iqs.get(om)[2], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dOut_iqs.get(om)[0], 1.0);
                    c.addToRHS(vp.auxQ_dOut.get(om), M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(vp.aux_dOut_iqs.get(om)[2], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dOut_iqs.get(om)[1], 1.0);
                    c.addToRHS(vp.auxQ_dOut.get(om), -M);
                    executor.addConstraint(c);


                    //d_ij: <-(pl.3)
                    c = new GurobiConstraint();
                    c.setName("d_ij: <-");
                    c.addToLHS(vp.dInOut_cqs[1], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dIn_iqs.get(om)[2], Math.sqrt(2));
                    c.addToRHS(vp.aux_dIn_iqs.get(om)[3], 1.0);
                    c.addToRHS(vp.inOutCnn_qs.get(om)[1], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    //auxiliary
                    c = new GurobiConstraint();
                    c.setName("d_ij: <-:aux_X");
                    c.addToLHS(vpN.x, 1.0);
                    c.addToLHS(vp.oCoordinate_iqs.get(om)[0], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_dIn_iqs.get(om)[4], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_dIn_iqs.get(om)[0], vp.aux_dIn_iqs.get(om)[4], "v" + virtualPointVars.indexOf(vp) + om.getName() + "absX_dIn");
                    c = new GurobiConstraint();
                    c.setName("d_ij: <-:aux_Y");
                    c.addToLHS(vpN.y, 1.0);
                    c.addToLHS(vp.oCoordinate_iqs.get(om)[1], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_dIn_iqs.get(om)[5], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_dIn_iqs.get(om)[1], vp.aux_dIn_iqs.get(om)[5], "v" + virtualPointVars.indexOf(vp) + om.getName() + "absY_dIn");
                    c = new GurobiConstraint();
                    c.setName("d_ij: <-:aux_XY");
                    c.addToLHS(vp.aux_dIn_iqs.get(om)[0], 1.0);
                    c.addToLHS(vp.aux_dIn_iqs.get(om)[1], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_dIn_iqs.get(om)[6], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_dIn_iqs.get(om)[3], vp.aux_dIn_iqs.get(om)[6], "v" + virtualPointVars.indexOf(vp) + om.getName() + "absXY_dIn");
                    //Min_linear
                    c = new GurobiConstraint();
                    c.addToLHS(vp.aux_dIn_iqs.get(om)[2], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dIn_iqs.get(om)[0], 1.0);
                    c.addToRHS(vp.auxQ_dIn.get(om), M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(vp.aux_dIn_iqs.get(om)[2], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dIn_iqs.get(om)[1], 1.0);
                    c.addToRHS(vp.auxQ_dIn.get(om), -M);
                    executor.addConstraint(c);

                }


            }

            /*
            Connection with Slaves
             */
            GurobiConstraint c_vsCnn = new GurobiConstraint();
            c_vsCnn.setSense('=');
            c_vsCnn.setRHSConstant(1.0);
            executor.addConstraint(c_vsCnn);

            for (int j = 0; j < slaves.size(); ++j) {
                PseudoBase sv = slaves.get(j);
                //sv.1
                c_vsCnn.addToLHS(vp.vsCnn_q.get(sv), 1.0);

                //sv.3
                c = new GurobiConstraint();
                c.setName("sv.3_geq");
                c.addToLHS(vp.vs_corDist_cq, 1.0);
                c.setSense('>');
                c.addToRHS(vp.vs_dist_cq.get(sv), 1.0);
                c.addToRHS(vp.vsCnn_q.get(sv), M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("sv.3_leq");
                c.addToLHS(vp.vs_corDist_cq, 1.0);
                c.setSense('<');
                c.addToRHS(vp.vs_dist_cq.get(sv), 1.0);
                c.addToRHS(vp.vsCnn_q.get(sv), -M);
                c.setRHSConstant(M);
                executor.addConstraint(c);


                //todo:d_i_sj < q_i_sj * M
//                c = new GurobiConstraint();
//                c.addToLHS(vp.vs_dist_cq.get(sv), 1.0);
//                c.setSense('<');
//                c.addToRHS(vp.vsCnn_q.get(sv), M);
//                executor.addConstraint(c);

                /*
                d_is_j
                VVVV
                 */

                c = new GurobiConstraint();
                c.setName("d_is_j_woDetour_>");
                c.addToLHS(vp.vs_dist_cq.get(sv), 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_vsDist_iqs.get(sv)[2], Math.sqrt(2));
                c.addToRHS(vp.aux_vsDist_iqs.get(sv)[3], 1.0);
                //todo
                c.addToRHS(vp.vsCnn_q.get(sv), M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);
                //aux
                c = new GurobiConstraint();
                c.setName("d_is_j_woDetour:aux_X");
                c.addToLHS(vp.x, 1.0);
                c.setLHSConstant(-sv.getX());
                c.setSense('=');
                c.addToRHS(vp.aux_vsDist_iqs.get(sv)[4], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_vsDist_iqs.get(sv)[0], vp.aux_vsDist_iqs.get(sv)[4], "|vi.x-sj.x|" + "v" + virtualPointVars.indexOf(vp) + "->" + sv.getName());
                c = new GurobiConstraint();
                c.setName("d_is_j_woDetour:aux_Y");
                c.addToLHS(vp.y, 1.0);
                c.setLHSConstant(-sv.getY());
                c.setSense('=');
                c.addToRHS(vp.aux_vsDist_iqs.get(sv)[5], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_vsDist_iqs.get(sv)[1], vp.aux_vsDist_iqs.get(sv)[5], "|vi.y-sj.y|" + "v" + virtualPointVars.indexOf(vp) + "->" + sv.getName());
                c = new GurobiConstraint();
                c.setName("d_is_j_woDetour:aux_XY");
                c.addToLHS(vp.aux_vsDist_iqs.get(sv)[0], 1.0);
                c.addToLHS(vp.aux_vsDist_iqs.get(sv)[1], -1.0);
                c.setSense('=');
                c.addToRHS(vp.aux_vsDist_iqs.get(sv)[6], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_vsDist_iqs.get(sv)[3], vp.aux_vsDist_iqs.get(sv)[6], "||vi.x-sj.x|-|vi.y-sj.y||" + "v" + virtualPointVars.indexOf(vp) + "->" + sv.getName());
                //Min_linear
                c = new GurobiConstraint();
                c.addToLHS(vp.aux_vsDist_iqs.get(sv)[2], 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_vsDist_iqs.get(sv)[0], 1.0);
                c.addToRHS(vp.auxQ_vsDist.get(sv), M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.aux_vsDist_iqs.get(sv)[2], 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_vsDist_iqs.get(sv)[1], 1.0);
                c.addToRHS(vp.auxQ_vsDist.get(sv), -M);
                executor.addConstraint(c);
                /*
                AAAA
                d_i_sj
                 */



                //vs_cnn.4
                GurobiQuadConstraint c_vsOut = new GurobiQuadConstraint();
                c_vsOut.setName("vs_cnn.4");
                c_vsOut.setSense('=');
                c_vsOut.addToRHS(vp.vs_detour_qs.get(sv)[4], 1.0);
                executor.addConstraint(c_vsOut);
                //vs_pl
                GurobiConstraint c_vs_dij = new GurobiConstraint();
                c_vs_dij.addToLHS(vp.vs_dist_cq.get(sv), 1.0);
                c_vs_dij.setSense('>');
                c_vs_dij.addToRHS(vp.vs_dInOut_cqs.get(sv)[0], 1.0);
                c_vs_dij.addToRHS(vp.vs_dInOut_cqs.get(sv)[1], 1.0);
                c_vs_dij.addToRHS(vp.vs_detour_qs.get(sv)[4], M);
                c_vs_dij.setRHSConstant(-M);
                executor.addConstraint(c_vs_dij);

                //1.vs_ul->lr.3
                GurobiConstraint c_vsULtoLR_geq = new GurobiConstraint();
                c_vsULtoLR_geq.setName("vs_ul->lr.3_geq");
                c_vsULtoLR_geq.addToLHS(vp.vs_detour_qs.get(sv)[0], obstacles.size());
                c_vsULtoLR_geq.setSense('>');
                executor.addConstraint(c_vsULtoLR_geq);
                GurobiConstraint c_vsULtoLR_leq = new GurobiConstraint();
                c_vsULtoLR_leq.setName("vs_ul->lr.3_leq");
                c_vsULtoLR_leq.addToLHS(vp.vs_detour_qs.get(sv)[0], 1.0);
                c_vsULtoLR_leq.setSense('<');
                executor.addConstraint(c_vsULtoLR_leq);
                //2.vs_lr->ul.3
                GurobiConstraint c_vsLRtoUL_geq = new GurobiConstraint();
                c_vsLRtoUL_geq.setName("vs_lr->ul.3_geq");
                c_vsLRtoUL_geq.addToLHS(vp.vs_detour_qs.get(sv)[1], obstacles.size());
                c_vsLRtoUL_geq.setSense('>');
                executor.addConstraint(c_vsLRtoUL_geq);
                GurobiConstraint c_vsLRtoUL_leq = new GurobiConstraint();
                c_vsLRtoUL_leq.setName("vs_lr->ul.3_leq");
                c_vsLRtoUL_leq.addToLHS(vp.vs_detour_qs.get(sv)[1], 1.0);
                c_vsLRtoUL_leq.setSense('<');
                executor.addConstraint(c_vsLRtoUL_leq);
                //3.vs_ur->ll.3
                GurobiConstraint c_vsURtoLL_geq = new GurobiConstraint();
                c_vsURtoLL_geq.setName("vs_ur->ll.3_geq");
                c_vsURtoLL_geq.addToLHS(vp.vs_detour_qs.get(sv)[2], obstacles.size());
                c_vsURtoLL_geq.setSense('>');
                executor.addConstraint(c_vsURtoLL_geq);
                GurobiConstraint c_vsURtoLL_leq = new GurobiConstraint();
                c_vsURtoLL_leq.setName("vs_ur->ll.3_leq");
                c_vsURtoLL_leq.addToLHS(vp.vs_detour_qs.get(sv)[2], 1.0);
                c_vsURtoLL_leq.setSense('<');
                executor.addConstraint(c_vsURtoLL_leq);
                //4.vs_ll->ur.3
                GurobiConstraint c_vsLLtoUR_geq = new GurobiConstraint();
                c_vsLLtoUR_geq.setName("vs_ll->ur.3_geq");
                c_vsLLtoUR_geq.addToLHS(vp.vs_detour_qs.get(sv)[3], obstacles.size());
                c_vsLLtoUR_geq.setSense('>');
                executor.addConstraint(c_vsLLtoUR_geq);
                GurobiConstraint c_vsLLtoUR_leq = new GurobiConstraint();
                c_vsLLtoUR_leq.setName("vs_ll->ur.3_leq");
                c_vsLLtoUR_leq.addToLHS(vp.vs_detour_qs.get(sv)[3], 1.0);
                c_vsLLtoUR_leq.setSense('<');
                executor.addConstraint(c_vsLLtoUR_leq);

                for (Obstacle om : obstacles) {

                    c_vsULtoLR_geq.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[0], 1.0);
                    c_vsULtoLR_leq.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[0], 1.0);

                    c_vsLRtoUL_geq.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[1], 1.0);
                    c_vsLRtoUL_leq.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[1], 1.0);

                    c_vsURtoLL_geq.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[2], 1.0);
                    c_vsURtoLL_leq.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[2], 1.0);

                    c_vsLLtoUR_geq.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[3], 1.0);
                    c_vsLLtoUR_leq.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[3], 1.0);

                    //vs_cnn.4
                    c_vsOut.addToLHS(vp.vs_inOutCnn_qs.get(sv).get(om)[0], vp.vs_relObstacles_qs.get(sv).get(om)[4], 1.0);

                    //1.vs_ul->lr
                    c = new GurobiConstraint();
                    c.setName("vs_aux_ul->lr");
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(om)[0], 1.0);
                    executor.addConstraint(c);
                    //todo
                    itmp = 0;
                    for (Obstacle on : om.get_bRObstacles()) {
                        //1.vs_ul->lr.1
                        c = new GurobiConstraint();
                        c.setName("vs_ul->lr.1");
                        c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[0], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.rel_qs.get(om)[0], 1.0);
                        c.setRHSConstant(sv.getPseudo_oRel_qs().get(on)[3] - 1.0);
                        executor.addConstraint(c);

                        itmp += sv.getPseudo_oRel_qs().get(on)[3];

                        if (!om.getName().equals(on.getName())) {
                            //1.(vs_ul->lr.2)
                            c = new GurobiConstraint();
                            c.setName("vs_ul->lr.2");
                            c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(on)[0], 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.rel_qs.get(om)[0], 1.0);
                            c.setRHSConstant(sv.getPseudo_oRel_qs().get(on)[3] - 1.0);
                            executor.addConstraint(c);
                        }
                    }
                    //todo: aux
                    c = new GurobiConstraint();
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[0], 1.0);
                    c.setSense('<');
                    c.setRHSConstant(itmp);
                    executor.addConstraint(c);


                    //2.vs_lr->ul
                    c = new GurobiConstraint();
                    c.setName("vs_aux_lr->ul");
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[1], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(om)[3], 1.0);
                    executor.addConstraint(c);
                    itmp = 0;
                    for (Obstacle on : om.get_tLObstacles()) {
                        //2.(vs_lr->ul.1)
                        c = new GurobiConstraint();
                        c.setName("vs_lr->ul.1");
                        c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[1], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.rel_qs.get(om)[3], 1.0);
                        c.setRHSConstant(sv.getPseudo_oRel_qs().get(on)[0] - 1.0);
                        executor.addConstraint(c);

                        //todo
                        itmp += sv.getPseudo_oRel_qs().get(on)[0];

                        if (!om.getName().equals(on.getName())) {
                            //2.(vs_lr->ul.2)
                            c = new GurobiConstraint();
                            c.setName("(vs_lr->ul.2)");
                            c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(on)[1], 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.rel_qs.get(om)[3], 1.0);
                            c.setRHSConstant(sv.getPseudo_oRel_qs().get(on)[0] - 1.0);
                            executor.addConstraint(c);
                        }
                    }
                    //todo: aux
                    c = new GurobiConstraint();
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[1], 1.0);
                    c.setSense('<');
                    c.setRHSConstant(itmp);
                    executor.addConstraint(c);

                    //3.vs_ur->ll
                    c = new GurobiConstraint();
                    c.setName("vs_aux_ur->ll");
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[2], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(om)[1], 1.0);
                    executor.addConstraint(c);
                    itmp = 0;
                    for (Obstacle on : om.get_bLObstacles()) {
                        //3.vs_ur->ll.1
                        c = new GurobiConstraint();
                        c.setName("vs_ur->ll");
                        c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[2], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.rel_qs.get(om)[1], 1.0);
                        c.setRHSConstant(sv.getPseudo_oRel_qs().get(on)[2] - 1.0);
                        executor.addConstraint(c);

                        //todo:
                        itmp += sv.getPseudo_oRel_qs().get(on)[2];

                        if (!om.getName().equals(on.getName())) {
                            //3.(vs_ur->ll.2)
                            c = new GurobiConstraint();
                            c.setName("(vs_ur->ll.2)");
                            c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(on)[2], 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.rel_qs.get(om)[1], 1.0);
                            c.setRHSConstant(sv.getPseudo_oRel_qs().get(on)[2] - 1.0);
                            executor.addConstraint(c);
                        }
                    }
                    //todo:aux
                    c = new GurobiConstraint();
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[2], 1.0);
                    c.setSense('<');
                    c.setRHSConstant(itmp);
                    executor.addConstraint(c);


                    //4.vs_ll->ur
                    c = new GurobiConstraint();
                    c.setName("vs_aux_ll->ur");
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[3], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.rel_qs.get(om)[2], 1.0);
                    executor.addConstraint(c);
                    itmp = 0;
                    for (Obstacle on : om.get_tRObstacles()) {
                        //4.vs_ll->ur.1
                        c = new GurobiConstraint();
                        c.setName("vs_ll->ur.1");
                        c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[3], 1.0);
                        c.setSense('>');
                        c.addToRHS(vp.rel_qs.get(om)[2], 1.0);
                        c.setRHSConstant(sv.getPseudo_oRel_qs().get(on)[1] - 1.0);
                        executor.addConstraint(c);

                        //todo
                        itmp += sv.getPseudo_oRel_qs().get(on)[1];

                        if (!om.getName().equals(on.getName())) {
                            //4.vs_ll->ur.2
                            c = new GurobiConstraint();
                            c.setName("vs_ll->ur.2");
                            c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(on)[3], 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.rel_qs.get(om)[2], 1.0);
                            c.setRHSConstant(sv.getPseudo_oRel_qs().get(on)[1] - 1.0);
                            executor.addConstraint(c);
                        }
                    }
                    //todo
                    c = new GurobiConstraint();
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[3], 1.0);
                    c.setSense('<');
                    c.setRHSConstant(itmp);
                    executor.addConstraint(c);


                    //vs_aux.1
                    c = new GurobiConstraint();
                    c.setName("vs_aux.1.1");
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[0], 1.0);
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[1], 1.0);
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[2], 1.0);
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[3], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[4], 4.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("vs_aux.1.2");
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[0], 1.0);
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[1], 1.0);
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[2], 1.0);
                    c.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[3], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[4], 1.0);
                    executor.addConstraint(c);


                    //vs_aux.2
                    c = new GurobiConstraint();
                    c.setName("vs_aux.2.1");
                    c.addToLHS(vp.vs_detour_qs.get(sv)[0], 1.0);
                    c.addToLHS(vp.vs_detour_qs.get(sv)[1], 1.0);
                    c.addToLHS(vp.vs_detour_qs.get(sv)[2], 1.0);
                    c.addToLHS(vp.vs_detour_qs.get(sv)[3], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.vs_detour_qs.get(sv)[4], 4.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("vs_aux.2.2");
                    c.addToLHS(vp.vs_detour_qs.get(sv)[0], 1.0);
                    c.addToLHS(vp.vs_detour_qs.get(sv)[1], 1.0);
                    c.addToLHS(vp.vs_detour_qs.get(sv)[2], 1.0);
                    c.addToLHS(vp.vs_detour_qs.get(sv)[3], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.vs_detour_qs.get(sv)[4], 1.0);
                    executor.addConstraint(c);

                    //vs_cnn_1
                    c = new GurobiConstraint();
                    c.setName("vs_cnn_1");
                    c.addToLHS(vp.vs_corner_qs.get(sv).get(om)[0], 1.0);
                    c.addToLHS(vp.vs_corner_qs.get(sv).get(om)[1], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[0], 1.0);
                    c.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[1], 1.0);
                    executor.addConstraint(c);
                    //vs_cnn_2
                    c = new GurobiConstraint();
                    c.setName("vs_cnn_2");
                    c.addToLHS(vp.vs_corner_qs.get(sv).get(om)[2], 1.0);
                    c.addToLHS(vp.vs_corner_qs.get(sv).get(om)[3], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[2], 1.0);
                    c.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[3], 1.0);
                    executor.addConstraint(c);

                    //vs_cnn.1
                    c = new GurobiConstraint();
                    c.setName("vs_cnn.1");
                    c.addToLHS(vp.vs_omOnCnn_q.get(sv).get(om).get(om), 1.0);
                    c.setSense('=');
                    c.setRHSConstant(0.0);
                    executor.addConstraint(c);

                    //vs_cnn.3_1
                    GurobiQuadConstraint c_vsOutCnn = new GurobiQuadConstraint();
                    c_vsOutCnn.setName("vs_cnn.3_1");
                    c_vsOutCnn.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[4], 1.0);
                    c_vsOutCnn.setSense('=');
                    c_vsOutCnn.addToRHS(vp.vs_inOutCnn_qs.get(sv).get(om)[1], 1.0);
                    executor.addConstraint(c_vsOutCnn);
                    //vs_cnn.3_2
                    GurobiQuadConstraint c_vsInCnn = new GurobiQuadConstraint();
                    c_vsInCnn.setName("vs_cnn.3_2");
                    c_vsInCnn.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[4], 1.0);
                    c_vsInCnn.setSense('=');
                    c_vsInCnn.addToRHS(vp.vs_inOutCnn_qs.get(sv).get(om)[0], 1.0);
                    executor.addConstraint(c_vsInCnn);


                    //vs_cnn.5
                    GurobiConstraint c_vsCnnAux1 = new GurobiConstraint();
                    c_vsCnnAux1.setName("vs_cnn.5_1");
                    c_vsCnnAux1.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[4], 1.0);
                    c_vsCnnAux1.setSense('>');
                    executor.addConstraint(c_vsCnnAux1);
                    GurobiConstraint c_vsCnnAux2 = new GurobiConstraint();
                    c_vsCnnAux2.setName("vs_cnn.5_2");
                    c_vsCnnAux2.addToLHS(vp.vs_relObstacles_qs.get(sv).get(om)[4], 1.0);
                    c_vsCnnAux2.setSense('>');
                    executor.addConstraint(c_vsCnnAux2);

                    //vs: Om <-> On
                    for (Obstacle on : obstacles) {
                        if (!om.getName().equals(on.getName())) {

                            c_vsCnnAux1.addToRHS(vp.vs_omOnCnn_q.get(sv).get(om).get(on), 1.0);
                            c_vsCnnAux2.addToRHS(vp.vs_omOnCnn_q.get(sv).get(on).get(om), 1.0);


                            //vs_cnn.2
                            c = new GurobiConstraint();
                            c.setName("vs_cnn.2");
                            c.addToLHS(vp.vs_omOnCnn_q.get(sv).get(om).get(on), 1.0);
                            c.addToLHS(vp.vs_omOnCnn_q.get(sv).get(on).get(om), 1.0);
                            c.setSense('<');
                            c.setRHSConstant(1.0);
                            executor.addConstraint(c);

                            //cnn.3_1
                            c_vsOutCnn.addToRHS(vp.vs_omOnCnn_q.get(sv).get(om).get(on), vp.vs_relObstacles_qs.get(sv).get(on)[4], 1.0);
                            //cnn.3_2
                            c_vsInCnn.addToRHS(vp.vs_omOnCnn_q.get(sv).get(on).get(om), vp.vs_relObstacles_qs.get(sv).get(om)[4], 1.0);

                            //d_i_sj_omOn
                            c = new GurobiConstraint();
                            c.setName("d_i_sj_omOn");
                            c.addToLHS(vp.vs_dOmOn_cq.get(sv).get(om).get(on), 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[2], Math.sqrt(2));
                            c.addToRHS(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[3], 1.0);
                            c.addToRHS(vp.vs_omOnCnn_q.get(sv).get(om).get(on), M);
                            c.setRHSConstant(-M);
                            executor.addConstraint(c);
                            //aux
                            c = new GurobiConstraint();
                            c.setName("d_i_sj_omOn_auxX");
                            c.addToLHS(vp.oCoordinate_iqs.get(om)[0], 1.0);
                            c.addToLHS(vp.oCoordinate_iqs.get(on)[0], -1.0);
                            c.setSense('=');
                            c.addToRHS(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[4], 1.0);
                            executor.addConstraint(c);
                            executor.addGenConstraintAbs(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[0], vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[4], "v" + virtualPointVars.indexOf(vp) + sv.getName() + om.getName() + "->" + on.getName() + "d_i_sj_omOn_absX");
                            c = new GurobiConstraint();
                            c.setName("d_i_sj_omOn_auxY");
                            c.addToLHS(vp.oCoordinate_iqs.get(om)[1], 1.0);
                            c.addToLHS(vp.oCoordinate_iqs.get(on)[1], -1.0);
                            c.setSense('=');
                            c.addToRHS(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[5], 1.0);
                            executor.addConstraint(c);
                            executor.addGenConstraintAbs(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[1], vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[5], "v" + virtualPointVars.indexOf(vp) + sv.getName() + om.getName() + "->" + on.getName() + "d_i_sj_omOn_absY");
                            c = new GurobiConstraint();
                            c.addToLHS(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[0], 1.0);
                            c.addToLHS(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[1], -1.0);
                            c.setSense('=');
                            c.addToRHS(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[6], 1.0);
                            executor.addConstraint(c);
                            executor.addGenConstraintAbs(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[3], vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[6], "v" + virtualPointVars.indexOf(vp) + sv.getName() + om.getName() + "->" + on.getName() + "d_i_sj_omOn_absXY");
                            //Min
                            c = new GurobiConstraint();
                            c.addToLHS(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[2], 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[0], 1.0);
                            c.addToRHS(vp.auxQ_vsdOmOn.get(sv).get(om).get(on), M);
                            c.setRHSConstant(-M);
                            executor.addConstraint(c);
                            c = new GurobiConstraint();
                            c.addToLHS(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[2], 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.aux_vsdOmOn_iqs.get(sv).get(om).get(on)[1], 1.0);
                            c.addToRHS(vp.auxQ_vsdOmOn.get(sv).get(om).get(on), -M);
                            executor.addConstraint(c);
                            //executor.addGenConstraintMin(vp.aux_vsdOmOn_cqs.get(sv).get(om).get(on)[2], new GurobiVariable[]{vp.aux_vsdOmOn_cqs.get(sv).get(om).get(on)[0], vp.aux_vsdOmOn_cqs.get(sv).get(om).get(on)[1]}, tolerance, "d_i_sj_omOn_MIN");

                            //Min_2
                            /*qc = new GurobiQuadConstraint();
                            qc.addToLHS(vp.aux_vsdOmOn_cqs.get(sv).get(om).get(on)[2], 1.0);
                            qc.setSense('=');
                            qc.addToRHS(vp.aux_vsdOmOn_cqs.get(sv).get(om).get(on)[0], vp.auxQ_vsdOmOn.get(sv).get(om).get(on), 1.0);
                            qc.addToRHS(vp.aux_vsdOmOn_cqs.get(sv).get(om).get(on)[1], vp.auxQ_vsdOmOn.get(sv).get(om).get(on), -1.0);
                            qc.addToRHS(vp.aux_vsdOmOn_cqs.get(sv).get(om).get(on)[1], 1.0);
                            executor.addConstraint(qc);
                            c = new GurobiConstraint();
                            c.addToLHS(vp.aux_vsdOmOn_cqs.get(sv).get(om).get(on)[0], 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.aux_vsdOmOn_cqs.get(sv).get(om).get(on)[1], 1.0);
                            c.addToRHS(vp.auxQ_vsdOmOn.get(sv).get(om).get(on), -M);
                            executor.addConstraint(c);*/

                            //pl
                            c_vs_dij.addToRHS(vp.vs_dOmOn_cq.get(sv).get(om).get(on), 1.0);

                        }
                    }



                    //vs_cor.x
                    c = new GurobiConstraint();
                    c.setName("vs_cor.x_leq");
                    c.addToLHS(vp.vs_oCoordinate_iqs.get(sv).get(om)[0], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[0], om.getMinX());
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[2], om.getMinX());
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[1], om.getMaxX());
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[3], om.getMaxX());
                    c.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[4], -M);
                    c.setRHSConstant(M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("vs_cor.x_geq");
                    c.addToLHS(vp.vs_oCoordinate_iqs.get(sv).get(om)[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[0], om.getMinX());
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[2], om.getMinX());
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[1], om.getMaxX());
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[3], om.getMaxX());
                    c.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[4], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);


                    //vs_cor.y
                    c = new GurobiConstraint();
                    c.setName("vs_cor.y_leq");
                    c.addToLHS(vp.vs_oCoordinate_iqs.get(sv).get(om)[1], 1.0);
                    c.setSense('<');
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[0], om.getMinY());
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[3], om.getMinY());
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[1], om.getMaxY());
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[2], om.getMaxY());
                    c.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[4], -M);
                    c.setRHSConstant(M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName("vs_cor.y_geq");
                    c.addToLHS(vp.vs_oCoordinate_iqs.get(sv).get(om)[1], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[0], om.getMinY());
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[3], om.getMinY());
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[1], om.getMaxY());
                    c.addToRHS(vp.vs_corner_qs.get(sv).get(om)[2], om.getMaxY());
                    c.addToRHS(vp.vs_relObstacles_qs.get(sv).get(om)[4], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);

                    //d_i_sj_->
                    c = new GurobiConstraint();
                    c.setName("d_i_sj_->");
                    c.addToLHS(vp.vs_dInOut_cqs.get(sv)[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_vsdOut_iqs.get(sv).get(om)[2], Math.sqrt(2));
                    c.addToRHS(vp.aux_vsdOut_iqs.get(sv).get(om)[3], 1.0);
                    c.addToRHS(vp.vs_inOutCnn_qs.get(sv).get(om)[0], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    //aux
                    c = new GurobiConstraint();
                    c.setName("vs_dOut_auxX");
                    c.addToLHS(vp.x, 1.0);
                    c.addToLHS(vp.vs_oCoordinate_iqs.get(sv).get(om)[0], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_vsdOut_iqs.get(sv).get(om)[4], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_vsdOut_iqs.get(sv).get(om)[0], vp.aux_vsdOut_iqs.get(sv).get(om)[4], "v" + virtualPointVars.indexOf(vp) + sv.getName() + om.getName() + "vs_dOut_absX");
                    c = new GurobiConstraint();
                    c.setName("vs_dOut_auxY");
                    c.addToLHS(vp.y, 1.0);
                    c.addToLHS(vp.vs_oCoordinate_iqs.get(sv).get(om)[1], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_vsdOut_iqs.get(sv).get(om)[5], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_vsdOut_iqs.get(sv).get(om)[1], vp.aux_vsdOut_iqs.get(sv).get(om)[5], "v" + virtualPointVars.indexOf(vp) + sv.getName() + om.getName() + "vs_dOut_absY");
                    c = new GurobiConstraint();
                    c.setName("vs_dOut_auxXY");
                    c.addToLHS(vp.aux_vsdOut_iqs.get(sv).get(om)[4], 1.0);
                    c.addToLHS(vp.aux_vsdOut_iqs.get(sv).get(om)[5], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_vsdOut_iqs.get(sv).get(om)[6], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_vsdOut_iqs.get(sv).get(om)[3], vp.aux_vsdOut_iqs.get(sv).get(om)[6], "v" + virtualPointVars.indexOf(vp) + sv.getName() + om.getName() + "vs_dOut_absXY");
                    //Min
                    c = new GurobiConstraint();
                    c.addToLHS(vp.aux_vsdOut_iqs.get(sv).get(om)[2], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_vsdOut_iqs.get(sv).get(om)[0], 1.0);
                    c.addToRHS(vp.auxQ_vsdOut.get(sv).get(om), M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(vp.aux_vsdOut_iqs.get(sv).get(om)[2], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_vsdOut_iqs.get(sv).get(om)[1], 1.0);
                    c.addToRHS(vp.auxQ_vsdOut.get(sv).get(om), -M);
                    executor.addConstraint(c);


                    //d_i_sj_<-
                    c = new GurobiConstraint();
                    c.setName("d_i_sj_<-");
                    c.addToLHS(vp.vs_dInOut_cqs.get(sv)[1], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_vsdIn_iqs.get(sv).get(om)[2], Math.sqrt(2));
                    c.addToRHS(vp.aux_vsdIn_iqs.get(sv).get(om)[3], 1.0);
                    c.addToRHS(vp.vs_inOutCnn_qs.get(sv).get(om)[1], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    //aux
                    c = new GurobiConstraint();
                    c.setName("d_i_sj_<-_auxX");
                    c.addToLHS(vp.vs_oCoordinate_iqs.get(sv).get(om)[0], 1.0);
                    c.setLHSConstant(-sv.getX());
                    c.setSense('=');
                    c.addToRHS(vp.aux_vsdIn_iqs.get(sv).get(om)[4], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_vsdIn_iqs.get(sv).get(om)[0], vp.aux_vsdIn_iqs.get(sv).get(om)[4], "v" + virtualPointVars.indexOf(vp) + sv.getName() + om.getName() + "d_i_sj_<-_absX");
                    c = new GurobiConstraint();
                    c.setName("d_i_sj_<-_auxY");
                    c.addToLHS(vp.vs_oCoordinate_iqs.get(sv).get(om)[1], 1.0);
                    c.setLHSConstant(-sv.getY());
                    c.setSense('=');
                    c.addToRHS(vp.aux_vsdIn_iqs.get(sv).get(om)[5], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_vsdIn_iqs.get(sv).get(om)[1], vp.aux_vsdIn_iqs.get(sv).get(om)[5], "v" + virtualPointVars.indexOf(vp) + sv.getName() + om.getName() + "d_i_sj_<-_absY");
                    c = new GurobiConstraint();
                    c.setName("d_i_sj_<-_auxXY");
                    c.addToLHS(vp.aux_vsdIn_iqs.get(sv).get(om)[0], 1.0);
                    c.addToLHS(vp.aux_vsdIn_iqs.get(sv).get(om)[1], -1.0);
                    c.setSense('=');
                    c.addToRHS(vp.aux_vsdIn_iqs.get(sv).get(om)[6], 1.0);
                    executor.addConstraint(c);
                    executor.addGenConstraintAbs(vp.aux_vsdIn_iqs.get(sv).get(om)[3], vp.aux_vsdIn_iqs.get(sv).get(om)[6], "v" + virtualPointVars.indexOf(vp) + sv.getName() + om.getName() + "d_i_sj_<-_absXY");
                    //Min
                    c = new GurobiConstraint();
                    c.addToLHS(vp.aux_vsdIn_iqs.get(sv).get(om)[2], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_vsdIn_iqs.get(sv).get(om)[0], 1.0);
                    c.addToRHS(vp.auxQ_vsdIn.get(sv).get(om), M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.addToLHS(vp.aux_vsdIn_iqs.get(sv).get(om)[2], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_vsdIn_iqs.get(sv).get(om)[1], 1.0);
                    c.addToRHS(vp.auxQ_vsdIn.get(sv).get(om), -M);
                    executor.addConstraint(c);

                }


            }


            /*
            Index 0 -- vps.size() - 1: without obstacles.
             */
            if (virtualPointVars.indexOf(vp) < virtualPointVars.size() - 1) {
                vpN = virtualPointVars.get(virtualPointVars.indexOf(vp) + 1);

                //busLength
                c_busLength.addToRHS(vp.dist_cq, 1.0);

                /*
                d_ij without Detour
                VVVV
                 */
                c = new GurobiConstraint();
                c.setName("d_ij:woDetour");
                c.addToLHS(vp.dist_cq, 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_dist_iqs[2], Math.sqrt(2));
                c.addToRHS(vp.aux_dist_iqs[3], 1.0);
                executor.addConstraint(c);
                //aux
                c = new GurobiConstraint();
                c.setName("d_ij:woDetour:aux_x");
                c.addToLHS(vp.x, 1.0);
                c.addToLHS(vpN.x, -1.0);
                c.setSense('=');
                c.addToRHS(vp.aux_dist_iqs[4], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_dist_iqs[0], vp.aux_dist_iqs[4], "|vi.x-vj.x|_" + virtualPointVars.indexOf(vp));
                c = new GurobiConstraint();
                c.setName("d_ij:woDetour:aux_Y");
                c.addToLHS(vp.y, 1.0);
                c.addToLHS(vpN.y, -1.0);
                c.setSense('=');
                c.addToRHS(vp.aux_dist_iqs[5], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_dist_iqs[1], vp.aux_dist_iqs[5], "|vi.y-j.y|_" + virtualPointVars.indexOf(vp));
                //Min
                c = new GurobiConstraint();
                c.addToLHS(vp.aux_dist_iqs[2], 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_dist_iqs[0], 1.0);
                c.addToRHS(vp.auxQ_dist, M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.addToLHS(vp.aux_dist_iqs[2], 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_dist_iqs[1], 1.0);
                c.addToRHS(vp.auxQ_dist, -M);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName("d_ij:woDetour:aux_XY");
                c.addToLHS(vp.aux_dist_iqs[0], 1.0);
                c.addToLHS(vp.aux_dist_iqs[1], -1.0);
                c.setSense('=');
                c.addToRHS(vp.aux_dist_iqs[6], 1.0);
                executor.addConstraint(c);
                executor.addGenConstraintAbs(vp.aux_dist_iqs[3], vp.aux_dist_iqs[6], "||vi.x-vj.x|-|vi.y-j.y||_" + virtualPointVars.indexOf(vp));
                /*
                AAAA
                d_ij
                 */




            }


        }


    }


    public void buildVars(ArrayList<Obstacle> obstacles, ArrayList<VirtualPointVar> virtualPointVars, ArrayList<PseudoBase> slaves, PseudoBase master) {

        GurobiVariable q, cq;

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
//        //lb_x
//        if (lb_x <= 0) {
//            lb_x *= 2;
//        } else {
//            lb_x *= 0.5;
//        }
//        //lb_y
//        if (lb_y <= 0) {
//            lb_y *= 2;
//        } else {
//            lb_y *= 0.5;
//        }
//        //ub_x
//        if (ub_x >= 0) {
//            ub_x *= 1.2;
//        } else {
//            ub_x *= 0.5;
//        }
//        //ub_y
//        if (ub_y >= 0) {
//            ub_y *= 1.2;
//        } else {
//            ub_y *= 0.5;
//        }

        int lb = Math.min(lb_x, lb_y);
        int ub = Math.max(ub_x, ub_y);



        /*
        initialize virtual points
         */
        for (int i = 0; i < slaves.size(); ++i) {
            VirtualPointVar vp = new VirtualPointVar();
            virtualPointVars.add(vp);
            vp.x = new GurobiVariable(GRB.INTEGER, lb_x, ub_x, "x" + i);
//            vp.x = new GurobiVariable(GRB.INTEGER, 3, 3, "x" + i);
            executor.addVariable(vp.x);
            vp.y = new GurobiVariable(GRB.INTEGER, lb_y, ub_y, "y" + i);
//            vp.y = new GurobiVariable(GRB.INTEGER, 4, 4, "y" + i);
            executor.addVariable(vp.y);


            for (Obstacle om : obstacles) {

                /*
                Regarding Master
                 */
                if (i == 0) {
                    /*
                    VM_relObstacles_qs
                    0: ul->lr
                    1: lr->ul
                    2: ur->ll
                    3: ll->ur
                    4: relative obstacle: aux.1/cnn.4
                     */
                    vp.vm_relObstacles_qs.put(om, buildBinaryVar(om.getName() + "_vm_relObstacles_qs", 5));

                    /*
                    VM_corner_qs
                    0: ll
                    1: ur
                    2: ul
                    3: lr
                     */
                    vp.vm_corner_qs.put(om, buildBinaryVar(om.getName() + "_vm_corner_qs_", 4));

                    /*
                    VM_inOutCnn_qs
                    0: vi->
                    1: vj<-
                     */
                    vp.vm_inOutCnn_qs.put(om, buildBinaryVar(om.getName() + "_vm_inOutCnn_qs_", 2));

                    /*
                    VM_oCoordinate_iqs
                    0: x_m
                    1: y_m
                     */
                    vp.vm_oCoordinate_iqs.put(om, buildBinaryVar(om.getName() + "_vm_oCoordinate_iqs_", 2));

                    /*
                    Auxiliary absolute values: aux_vmdOut_iqs
                    0: |x_m - ms.x|
                    1: |y_m - ms.y|
                    2: min(|x_m - ms.x|, |y_m - ms.y|)
                    3: ||x_m - ms.x| - |y_m - ms.y||
                    4: x_m - ms.x
                    5: y_m - ms.y
                    6: |x_m - ms.x| - |y_m - ms.y|
                     */
                    vp.aux_vmdOut_iqs.put(om, buildAuxIntVar("v_" + i + ";" + om.getName() + "_aux_vmdOut_iqs_"));
                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + om.getName() + "_auxQ_vmdOut");
                    executor.addVariable(q);
                    vp.auxQ_vmdOut.put(om, q);

                    /*
                    Auxiliary absolute values: aux_vmdIn_iqs
                    0: |x_n - ms.x|
                    1: |y_n - ms.y|
                    2: min(|x_n - ms.x|, |y_n - ms.y|)
                    3: ||x_n - ms.x| - |y_n - ms.y||
                    4: x_n - ms.x
                    5: y_n - ms.y
                    6: |x_n - ms.x| - |y_n - ms.y|
                     */
                    vp.aux_vmdIn_iqs.put(om, buildAuxIntVar("v_" + i + ";" + om.getName() + "_aux_vmdIn_iqs_"));
                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + om.getName() + "_auxQ_vmdIn");
                    executor.addVariable(q);
                    vp.auxQ_vmdIn.put(om, q);

                    /*
                    VM_omOnCnn_q
                    q_i_vj^m->n
                     */
                    Map<Obstacle, GurobiVariable> vm_omOnCnn_qMap = new HashMap<>();
                    /*
                    VM_dOmOn_cq
                    0: d_m->n
                     */
                    Map<Obstacle, GurobiVariable> vm_dOmOn_cqMap = new HashMap<>();
                    /*
                    Auxiliary absolute values: aux_vmdOmOn_iqs
                    0: |x_m - x_n|
                    1: |y_m - y_n|
                    2: min(|x_m - x_n|, |y_m - y_n|)
                    3: ||x_m - x_n| - |y_m - y_n||
                    4: x_m - x_n
                    5: y_m - y_n
                    6: |x_m - x_n| - |y_m - y_n|
                     */
                    Map<Obstacle, GurobiVariable[]> aux_vmdOmOn_iqsMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable> auxQ_vmdOmOnMap = new HashMap<>();
                    for (Obstacle on : obstacles) {
                        q = new GurobiVariable(GRB.BINARY, 0, 1, "_vm_omOnCnn_q");
                        executor.addVariable(q);
                        vm_omOnCnn_qMap.put(on, q);

                        /*cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "_vm_dOmOn_cq");
                        executor.addVariable(cq);*/
                        cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "_vm_dOmOn_cq");
                        executor.addVariable(cq);
                        vm_dOmOn_cqMap.put(on, cq);

                        aux_vmdOmOn_iqsMap.put(on, buildAuxIntVar("v_" + i + ";" + om.getName() + "->" + on.getName() + "_aux_vmdOmOn_cqs_"));
                        q = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + om.getName() + "->" + on.getName() + "_auxQ_vmdOmOn");
                        executor.addVariable(q);
                        auxQ_vmdOmOnMap.put(on, q);
                    }
                    vp.vm_omOnCnn_q.put(om, vm_omOnCnn_qMap);
                    vp.vm_dOmOn_cq.put(om, vm_dOmOn_cqMap);
                    vp.aux_vmdOmOn_iqs.put(om, aux_vmdOmOn_iqsMap);
                    vp.auxQ_vmdOmOn.put(om, auxQ_vmdOmOnMap);


                }



                /*
                Non-overlapping
                0: nonL
                1: nonR
                2: nonA
                3: nonB
                 */
                vp.non_qs.put(om, buildBinaryVar("v_" + i + ";" + om.getName() + "_non_qs_", 4));

                /*
                dir_qs
                0: ul
                1: ur
                2: lr
                3: ll
                 */
                vp.dir_qs.put(om, buildBinaryVar("v_" + i + ";" + om.getName() + "_dir_qs_", 4));

                /*
                rel_qs
                0: o_tL
                1: o_tR
                2: o_bL
                3: o_bR
                 */
                vp.rel_qs.put(om, buildBinaryVar("v_" + i + ";" + om.getName() + "_rel_qs_", 4));
                /*
                rel_AusQs
                0: nonL * nonT
                1: nonR * nonT
                2: nonL * nonB
                3: nonR * nonB
                 */
                vp.rel_AusQs.put(om, buildBinaryVar("v_" + i + ";" + om.getName() + "_rel_AuxQs_", 4));

                /*
                o_vp_relObstacles_qs
                0: ul->lr
                1: lr->ul
                2: ur->ll
                3: ll->ur
                4: relative obstacle: aux.2
                 */
                vp.relObstacles_qs.put(om, buildBinaryVar("v_" + i + ";" + om.getName() + "_relObstacles_qs_", 5));


                /*
                corner_qs
                0: ll
                1: ur
                2: ul
                3: lr
                 */
                vp.corner_qs.put(om, buildBinaryVar("v_" + i + ";" + om.getName() + "_corner_qs_", 4));

                /*
                inOutCnn_qs
                0: ->
                1: <-
                */
                vp.inOutCnn_qs.put(om, buildBinaryVar("v_" + i + ";" + om.getName() + "_inOutCnn_qs_", 2));


                /*
                oCoordinate_iqs
                0: x_m
                1: y_m
                 */
                vp.oCoordinate_iqs.put(om, buildIntVar(-M, M, "v_" + i + ";" + om.getName() + "_oCoordinate_iqs_", 2));



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
                Auxiliary absolute values: aux_dOmOn_iqs
                0: |x_m - x_n|
                1: |y_m - y_n|
                2: min(|x_m - x_n|, |y_m - y_n|)
                3: ||x_m - x_n| - |y_m - y_n||
                4: x_m - x_n
                5: y_m - y_n
                6: |x_m - x_n| - |y_m - y_n|
                 */
                Map<Obstacle, GurobiVariable[]> aux_dOmOn_iqsMap = new HashMap<>();
                Map<Obstacle, GurobiVariable> auxQ_dOmOnMap = new HashMap<>();
                for (Obstacle on : obstacles) {

                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + om.getName() + "-" + on.getName() + "_omOnCnn_q_");
                    executor.addVariable(q);
                    omOnCnn_qMap.put(on, q);

                    /*cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "v_" + i + ";" + om.getName() + "->" + on.getName() + "_dOmOn_cq_");
                    executor.addVariable(cq);*/
                    cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "v_" + i + ";" + om.getName() + "->" + on.getName() + "_dOmOn_cq_");
                    executor.addVariable(cq);
                    dOmOn_cqMap.put(on, cq);

                    //aux_dOmOn_iqsMap.put(on, buildContinuousVar(-M, M, "v_" + i + ";" + o.getName() + "->" + on.getName() + "_aux_dOmOn_iqs_", 7));
                    aux_dOmOn_iqsMap.put(on, buildAuxIntVar("v_" + i + ";" + om.getName() + "->" + on.getName() + "_aux_dOmOn_iqs_"));
                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + om.getName() + "->" + on.getName() + "_auxQ_dOmOn");
                    executor.addVariable(q);
                    auxQ_dOmOnMap.put(on, q);


                }
                vp.omOnCnn_q.put(om, omOnCnn_qMap);
                vp.dOmOn_cq.put(om, dOmOn_cqMap);
                vp.aux_dOmOn_iqs.put(om, aux_dOmOn_iqsMap);
                vp.auxQ_dOmOn.put(om, auxQ_dOmOnMap);



                /*
                Auxiliary absolute values: aux_dOut_iqs
                0: |vi.x - x_m|
                1: |vi.y - y_m|
                2: min(|vi.x - x_m|, |vi.y - y_m|)
                3: ||vi.x - x_m| - |vi.y - y_m||
                4: vi.x - x_m
                5: vi.y - y_m
                6: |vi.x - x_m| - |vi.y - y_m|
                 */
                //vp.aux_dOut_cqs.put(o, buildContinuousVar(-M, M, "v_" + i + ";" + o.getName() + "_aux_dOut_cqs_", 7));
                vp.aux_dOut_iqs.put(om, buildAuxIntVar("v_" + i + ";" + om.getName() + "_aux_dOut_iqs_"));
                q = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + om.getName() + "_auxQ_dOut");
                executor.addVariable(q);
                vp.auxQ_dOut.put(om, q);

                /*
                Auxiliary absolute values: aux_dIn_iqs
                0: |vi.x - x_m|
                1: |vi.y - y_m|
                2: min(|vi.x - x_m|, |vi.y - y_m|)
                3: ||vi.x - x_m| - |vi.y - y_m||
                4: vi.x - x_m
                5: vi.y - y_m
                6: |vi.x - x_m| - |vi.y - y_m|
                 */
                //vp.aux_dIn_cqs.put(o, buildContinuousVar(-M, M, "v_" + i + ";" + o.getName() + "_aux_dIn_cqs_", 7));
                vp.aux_dIn_iqs.put(om, buildAuxIntVar("v_" + i + ";" + om.getName() + "_aux_dIn_iqs_"));
                q = new GurobiVariable(GRB.BINARY, 0, 1, "v_" + i + ";" + om.getName() + "_auxQ_dIn");
                executor.addVariable(q);
                vp.auxQ_dIn.put(om, q);

            }

            if (i == 0) {
                /*
                VM_detour_qs
                0: ul->lr
                1: lr->ul
                2: ur->ll
                3: ll->ur
                4: q_ij^d: detour trigger: aux.2/cnn.4
                 */
                vp.vm_detour_qs = buildBinaryVar("vm_detour_qs_", 5);

                /*
                VM_dInOut_cqs
                0: d_i_ms->
                1: d_i_ms<-
                 */
                //vp.vm_dInOut_cqs = buildContinuousVar(0, M, "v" + i + "_vm_dInOut_cqs_", 2);
                vp.vm_dInOut_cqs = buildContinuousVar(0, M, "v" + i + "_vm_dInOut_cqs_", 2);

                /*
                Auxiliary absolute values: aux_vmDist_iqs
                0: |vi.x - ms.x|
                1: |vi.y - ms.y|
                2: min(|vi.x - ms.x|, |vi.y - ms.y|)
                3: ||vi.x - ms.x| - |vi.y - ms.y||
                4: vi.x - ms.x
                5: vi.y - ms.y
                6: |vi.x - ms.x| - |vi.y - ms.y|
                 */
                vp.aux_vmDist_iqs = buildAuxIntVar("v" + i + "_aux_vmDist_iqs_");
                vp.auxQ_vmDist = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_auxQ_vmDist");
                executor.addVariable(vp.auxQ_vmDist);

            }
            /*
            detour_qs
            0: ul->lr
            1: lr->ul
            2: ur->ll
            3: ll->ur
            4: detour trigger:
             */
            vp.detour_qs = buildBinaryVar("v" + i + "_detour_qs_", 5);

            /*
            dInOut_cqs
            0: d_->
            1: d_<-
             */
            //vp.dInOut_cqs = buildContinuousVar(0, M, "v" + i + "_dInOut_cqs_", 2);
            vp.dInOut_cqs = buildContinuousVar(0, M, "v" + i + "_dInOut_cqs_", 2);


            /*
            dist_cqs
            0: vv dist
            1: v.corrS dist
            2: vm dist (only for 1st vp)
             */
            /*if (i == 0) {
                vp.dist_cqs = buildContinuousVar(0, M, "v" + i + "dist_cqs", 3);
            } else {
                vp.dist_cqs = buildContinuousVar(0, M, "v" + i + "dist_cqs", 2);
            }*/

            /*
            dist_cq: vv dist
             */
            //vp.dist_cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "v" + i + "dist_cq");
            vp.dist_cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "v" + i + "dist_cq");
            executor.addVariable(vp.dist_cq);

            /*
            vs_corDist_cq: dist between virtualPoint and corresponding slave
             */
            //vp.vs_corDist_cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "v" + i + "corDist_cq");
            vp.vs_corDist_cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "v" + i + "corDist_cq");
            executor.addVariable(vp.vs_corDist_cq);

            /*
            Only for 1st vp: vm dist
             */
            if (i == 0){
//                vp.vm_dist_cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "vm_dist_cq");
                vp.vm_dist_cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "vm_dist_cq");
                executor.addVariable(vp.vm_dist_cq);
            }

            /*
            Auxiliary absolute values: aux_dist_iqs
            0: |vi.x-vj.x|
            1: |vi.y-j.y|
            2: min(|vi.x-vj.x|,|vi.y-j.y|)
            3: ||vi.x-vj.x|-|vi.y-j.y||
            4: vi.x-vj.x
            5: vi.y-j.y
            6: |vi.x-vj.x|-|vi.y-j.y|
             */
            //vp.aux_dist_cqs = buildContinuousVar(-M, M, "v" + i + "_aux_dist_cqs_", 7);
            vp.aux_dist_iqs = buildAuxIntVar("v" + i + "_aux_dist_iqs_");
            vp.auxQ_dist = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_auxQ_dist");
            executor.addVariable(vp.auxQ_dist);





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
                vs_inOutCnn_qs
                0: vi_s ->
                1: sj<-
                 */
                Map<Obstacle, GurobiVariable[]> vs_inOutCnn_qsMap = new HashMap<>();
                /*
                vs_oCoordinate_iqs
                0: x_m
                1: y_m
                */
                Map<Obstacle, GurobiVariable[]> vs_oCoordinate_iqsMap = new HashMap<>();
                /*
                vs_dOmOn_cq
                d_m->n
                 */
                Map<Obstacle, Map<Obstacle, GurobiVariable>> vs_dOmOn_cqMap = new HashMap<>();

                /*
                Auxiliary absolute values: aux_vsdOmOn_cqs
                0: |vi.x - sj.x|
                1: |vi.y - sj.y|
                2: min(|vi.x - sj.x|, |vi.y - sj.y|)
                3: ||vi.x - sj.x| - |vi.y - sj.y||
                4: vi.x - sj.x
                5: vi.y - sj.y
                6: |vi.x - sj.x| - |vi.y - sj.y|
                 */
                Map<Obstacle, Map<Obstacle, GurobiVariable[]>> aux_vsdOmOn_iqsMap = new HashMap<>();
                Map<Obstacle, Map<Obstacle, GurobiVariable>> auxQ_vsdOmOnMap = new HashMap<>();

                /*
                Auxiliary absolute values: aux_vsdOut_iqs
                0: |vi.x - x_m|
                1: |vi.y - y_m|
                2: min(|vi.x - x_m|, |vi.y - y_m|)
                3: ||vi.x - x_m| - |vi.y - y_m||
                4: vi.x - x_m
                5: vi.y - y_m
                6: |vi.x - x_m| - |vi.y - y_m|
                 */
                Map<Obstacle, GurobiVariable[]> aux_vsdOut_iqsMap = new HashMap<>();
                Map<Obstacle, GurobiVariable> auxQ_vsdOutMap = new HashMap<>();
                /*
                Auxiliary absolute values: aux_vsdIn_iqs
                0: |x_m - sj.x|
                1: |y_m - sj.y|
                2: min(|x_m - sj.x|, |y_m - sj.y|)
                3: ||x_m - sj.x| - |y_m - sj.y||
                4: x_m - sj.x
                5: y_m - sj.y
                6: |x_m - sj.x| - |vi.y - sj.y|
                 */
                Map<Obstacle, GurobiVariable[]> aux_vsdIn_iqsMap = new HashMap<>();
                Map<Obstacle, GurobiVariable> auxQ_vsdInMap = new HashMap<>();

                for (Obstacle o : obstacles) {
                    vs_relObstacles_qsMap.put(o, buildBinaryVar("v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_relObstacles_qs_", 5));
                    vs_corner_qsMap.put(o, buildBinaryVar("v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_corner_qs_", 4));


                    Map<Obstacle, GurobiVariable> vs_onCnn_qMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable> vs_dOn_cqsMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable[]> aux_vsdOn_iqsMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable> auxQ_vsdOnMap = new HashMap<>();
                    for (Obstacle other_o : obstacles) {
                        q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + o.getName() + ";" + sv.getName() + other_o.getName() + "_vs_omOnCnn_q");
                        executor.addVariable(q);
                        vs_onCnn_qMap.put(other_o, q);

                        /*cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "v" + i + ";" + o.getName() + ";" + sv.getName() + ";" + other_o.getName() +"_vs_dOmOn_cqs");
                        executor.addVariable(cq);*/
                        cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "v" + i + ";" + o.getName() + ";" + sv.getName() + ";" + other_o.getName() +"_vs_dOmOn_cqs");
                        executor.addVariable(cq);
                        vs_dOn_cqsMap.put(other_o, cq);

                        //aux_vsdOn_iqsMap.put(other_o, buildContinuousVar(-M, M, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_aux_vsdOut_iqs_", 7));
                        aux_vsdOn_iqsMap.put(other_o, buildAuxIntVar("v" + i + ";" + o.getName() + ";" + sv.getName() + other_o.getName() +"_aux_vsdOmOn_iqs_"));
                        q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + o.getName() + ";" + sv.getName() + other_o.getName() + "_auxQ_vsdOut");
                        executor.addVariable(q);
                        auxQ_vsdOnMap.put(other_o, q);
                    }

                    aux_vsdOmOn_iqsMap.put(o, aux_vsdOn_iqsMap);
                    auxQ_vsdOmOnMap.put(o, auxQ_vsdOnMap);
                    vs_omOnCnn_qMap.put(o, vs_onCnn_qMap);

                    vs_inOutCnn_qsMap.put(o, buildBinaryVar("v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_inCnn_q_", 2));


                    vs_oCoordinate_iqsMap.put(o, buildIntVar(-M, M, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_vs_oCoordinate_iqs_", 2));

                    vs_dOmOn_cqMap.put(o, vs_dOn_cqsMap);

                    //aux_vsdOut_iqsMap.put(o, buildContinuousVar(-M, M, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_aux_vsdOut_cqs_", 7));
                    aux_vsdOut_iqsMap.put(o, buildAuxIntVar("v" + i + ";" + o.getName() + ";" + sv.getName() + "_aux_vsdOut_cqs_"));
                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_auxQ_vsdOut");
                    executor.addVariable(q);
                    auxQ_vsdOutMap.put(o, q);

                    //aux_vsdIn_iqsMap.put(o, buildContinuousVar(-M, M, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_aux_vsdOut_iqs_", 7));
                    aux_vsdIn_iqsMap.put(o, buildAuxIntVar("v" + i + ";" + o.getName() + ";" + sv.getName() + "_aux_vsdOut_iqs_"));
                    q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + o.getName() + ";" + sv.getName() + "_auxQ_vsdOut");
                    executor.addVariable(q);
                    auxQ_vsdInMap.put(o, q);

                }
                vp.vs_relObstacles_qs.put(sv, vs_relObstacles_qsMap);
                vp.vs_corner_qs.put(sv, vs_corner_qsMap);
                vp.vs_omOnCnn_q.put(sv, vs_omOnCnn_qMap);
                vp.vs_inOutCnn_qs.put(sv, vs_inOutCnn_qsMap);
                vp.vs_oCoordinate_iqs.put(sv, vs_oCoordinate_iqsMap);
                vp.vs_dOmOn_cq.put(sv, vs_dOmOn_cqMap);
                vp.aux_vsdOut_iqs.put(sv, aux_vsdOut_iqsMap);
                vp.aux_vsdOmOn_iqs.put(sv, aux_vsdOmOn_iqsMap);
                vp.aux_vsdIn_iqs.put(sv, aux_vsdIn_iqsMap);
                vp.auxQ_vsdOut.put(sv, auxQ_vsdOutMap);
                vp.auxQ_vsdIn.put(sv, auxQ_vsdInMap);
                vp.auxQ_vsdOmOn.put(sv, auxQ_vsdOmOnMap);


                /*
                vs_detour_qs
                0: ul->lr
                1: lr->ul
                2: ur->ll
                3: ll->ur
                4: q_ij^d: detour trigger
                */
                vp.vs_detour_qs.put(sv, buildBinaryVar("v" + i + ";" + sv.getName() + "_vs_detour_qs_", 5));

                /*
                vs_dInOut_cqs
                1: d_vs->
                0: d_vs<-
                 */
//                vp.vs_dInOut_cqs.put(sv, buildContinuousVar(0, M, "v" + i + ";" + sv.getName() + "_vs_dInOut_cqs_", 2));
                vp.vs_dInOut_cqs.put(sv, buildContinuousVar(0, M, "v" + i + ";" + sv.getName() + "_vs_dInOut_cqs_", 2));

                /*
                Auxiliary absolute values: aux_vsDist_iqs
                0: |vi.x-sj.x|
                1: |vi.y-sj.y|
                2: min(|vi.x-sj.x|,|vi.y-sj.y|)
                3: ||vi.x-sj.x|-|vi.y-sj.y||
                4: vi.x-sj.x
                5: vi.y-sj.y
                6: |vi.x-sj.x|-|vi.y-sj.y|
                 */
                //vp.aux_vsDist_cqs.put(sv, buildContinuousVar(-M, M, "v" + i + ";" + sv.getName() + "_aux_vsDist_cqs_", 7));
                vp.aux_vsDist_iqs.put(sv, buildAuxIntVar("v" + i + ";" + sv.getName() + "_aux_vsDist_iqs_"));
                q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + ";" + sv.getName() + "_auxQ_vsDist");
                executor.addVariable(q);
                vp.auxQ_vsDist.put(sv, q);

                /*
                d_i_sj
                 */
                /*cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "v" + i + ";" + sv.getName() + "_d_i_sj");
                executor.addVariable(cq);*/
                cq = new GurobiVariable(GRB.CONTINUOUS, 0, M, "v" + i + ";" + sv.getName() + "_d_i_sj");
                executor.addVariable(cq);
                vp.vs_dist_cq.put(sv, cq);

            }


        }


    }

    private GurobiVariable[] buildAuxIntVar(String varName) {
        GurobiVariable[] qs = new GurobiVariable[7];
        for (int i = 0; i < 4; ++i) {
            qs[i] = new GurobiVariable(GRB.INTEGER, 0, M, varName + i);
            executor.addVariable(qs[i]);
        }
        for (int i = 4; i < 7; ++i) {
            qs[i] = new GurobiVariable(GRB.INTEGER, -M, M, varName + i);
            executor.addVariable(qs[i]);
        }
        return qs;
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
        oDir_q: UL, UR, LR, LL, L, R, T, B
         */
        int[] odir_q = new int[8];
        //UL
        if (base.getY() < base.getX() + o.getMaxY() - o.getMinX()) {
            odir_q[0] = 0;
        } else
            odir_q[0] = 1;
        //UR
        if (base.getY() <= -base.getX() + o.getMaxY() + o.getMaxX()) {
            odir_q[1] = 0;
        } else
            odir_q[1] = 1;
        //LR
        if (base.getY() <= base.getX() + o.getMinY() - o.getMaxX()) {
            odir_q[2] = 1;
        } else
            odir_q[2] = 0;
        //LL
        if (base.getY() < -base.getX() + o.getMinY() + o.getMinX()) {
            odir_q[3] = 1;
        } else
            odir_q[3] = 0;
        //L
        if (base.getX() < o.getMinX()) {
            odir_q[4] = 0;
        } else
            odir_q[4] = 1;
        //R
        if (base.getX() > o.getMaxX()) {
            odir_q[5] = 0;
        } else
            odir_q[5] = 1;
        //T
        if (base.getY() > o.getMaxY()) {
            odir_q[6] = 0;
        } else
            odir_q[6] = 1;
        //B
        if (base.getY() < o.getMinY()) {
            odir_q[7] = 0;
        } else
            odir_q[7] = 1;
        base.addToPseudo_oDir_qs(o, odir_q);

        /*
        oRel_q:
         */
        int[] orel_q = new int[4];
        //UpperLeft
        if (odir_q[3] + odir_q[1] + odir_q[4] * odir_q[6] == 0) {
            orel_q[0] = 1;
            base.addToOULd(o);
        } else orel_q[0] = 0;
        //UpperRight
        if (odir_q[0] + odir_q[2] + odir_q[5] * odir_q[6] == 0) {
            orel_q[1] = 1;
            base.addToOURd(o);
        } else orel_q[1] = 0;
        //LowerLeft
        if (odir_q[0] + odir_q[2] + odir_q[4] * odir_q[7] == 0) {
            orel_q[2] = 1;
            base.addToOLLd(o);
        } else orel_q[2] = 0;
        //LowerRight
        if (odir_q[1] + odir_q[3] + odir_q[5] * odir_q[7] == 0) {
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
        grbAsString.delete(grbAsString.length() - 1, grbAsString.length()).append("]");
        return grbAsString.toString();
    }

    public String convertGrbContArrayToString(GurobiVariable[] grbContArray) throws GRBException {
        StringBuilder grbAsString = new StringBuilder("[");
        for (GurobiVariable v : grbContArray) {
            grbAsString.append(v.getContResult() + " ");
        }
        grbAsString.delete(grbAsString.length() - 1, grbAsString.length()).append("]");
        return grbAsString.toString();
    }

}
