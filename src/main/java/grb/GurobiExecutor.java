/*
 * Copyright (c) 2021. Yushen Zhang
 *
 *   CONFIDENTIAL
 *   __________________
 *
 *   Yushen Zhang
 *   All Rights Reserved.
 *
 *   NOTICE:  All information contained herein is, and remains
 *   the property of Yushen Zhang and the Technical University
 *   of Munich, if applies. The intellectual and technical concepts contained
 *   herein are proprietary to Yushen Zhang and/or the Technical University
 *   of Munich and may be covered by European and Foreign Patents,
 *   patents in process, and are protected by trade secret or copyright law.
 *   Dissemination of this information or reproduction of this material
 *   is strictly forbidden unless prior written permission is obtained
 *   from Yushen Zhang.
 */

package grb;

import gurobi.*;

import java.util.ArrayList;

public class GurobiExecutor {

    private GRBEnv env;
    private GRBModel model;
    private String id;
    private ArrayList<GRBVar> vars;
    private ArrayList<GurobiVariable> variables;
    private ArrayList<GurobiConstraint> constraints;
    private GurobiObjConstraint objConstraint;

    public GurobiExecutor(String id) throws GRBException {
        this.id = id;
        this.env = new GRBEnv();
        /*try {
            File f= new File(this.id+".log");
            if(f.delete()){
                System.err.println("Old Log deleted.");
            }else{
                System.err.println("Old Log deletion failed.");
            }
        }catch (Exception e){
            System.err.println("Old Log not exist.");
        }*/
        this.env.set("logFile", id + ".log");
        this.env.set("LogToConsole","1");
        this.model = new GRBModel(env);
        this.vars = new ArrayList<>();
        this.variables = new ArrayList<>();
        this.constraints = new ArrayList<>();
    }

    /**
     *
     * @param presolve
     * @throws GRBException
     */
    public void setPresolve(int presolve) throws GRBException {
        this.model.set(GRB.IntParam.Presolve, presolve);
    }

    /**
     * Limits the total time expended (in seconds). Optimization returns with a TIME_LIMIT status if the limit is exceeded (see the Status Code section for further details).
     *
     * @param time Minimum value:	0 and	Maximum value:	Infinity
     * @throws GRBException Handled by client.
     */
    public void setTimeLimit(double time) throws GRBException {
        this.model.set(GRB.DoubleParam.TimeLimit, time);
    }

    /**
     * The MIP solver will terminate (with an optimal result) when the gap between the lower and upper objective bound is less than MIPGapAbs.
     *
     * @param gapAbs Minimum value:	0 and	Maximum value:	Infinity
     * @throws GRBException Handled by client.`
     */
    public void setMIPGapAbs(double gapAbs) throws GRBException {
        this.model.set(GRB.DoubleParam.MIPGapAbs, gapAbs);
    }

    /**
     * The MIP solver will terminate (with an optimal result) when the gap between the lower and upper objective bound is less than MIPGap times the absolute value of the incumbent objective value. More precisely, if <span>$</span>z_P<span>$</span> is the primal objective bound (i.e., the incumbent objective value, which is the upper bound for minimization problems), and <span>$</span>z_D<span>$</span> is the dual objective bound (i.e., the lower bound for minimization problems), then the MIP gap is defined as
     *
     * <span>$</span>gap = \vert z_P - z_D\vert / \vert z_P\vert<span>$</span>.
     * Note that if <span>$</span>z_P = z_D = 0<span>$</span>, then the gap is defined to be zero. If <span>$</span>z_P = 0<span>$</span> and <span>$</span>z_D \neq 0<span>$</span>, the gap is defined to be infinity.
     * <p>
     * For most models, <span>$</span>z_P<span>$</span> and <span>$</span>z_D<span>$</span> will have the same sign throughout the optimization process, and then the gap is monotonically decreasing. But if <span>$</span>z_P<span>$</span> and <span>$</span>z_D<span>$</span> have opposite signs, the relative gap may increase after finding a new incumbent solution, even though the absolute gap <span>$</span>\vert z_P - z_D\vert<span>$</span> has decreased.
     *
     * @param gap Minimum value:	0 and	Maximum value:	Infinity
     * @throws GRBException Handled by client.
     */
    public void setMIPGap(double gap) throws GRBException {
        this.model.set(GRB.DoubleParam.MIPGap, gap);
    }

    /**
     * Determines the amount of time spent in MIP heuristics. You can think of the value as the desired fraction of total MIP runtime devoted to heuristics (so by default, we aim to spend 5% of runtime on heuristics). Larger values produce more and better feasible solutions, at a cost of slower progress in the best bound.
     *
     * @param heuristics Minimum value:	0 and Maximum value: 1
     * @throws GRBException Handled by client.
     */
    public void setHeuristics(double heuristics) throws GRBException {
        this.model.set(GRB.DoubleParam.Heuristics, heuristics);
    }

    /**
     * Enables or disables solver output. Use LogFile and LogToConsole for finer-grain control. Setting OutputFlag to 0 is equivalent to setting LogFile to "" and LogToConsole to 0.
     *
     * @param flag Minimum value: 0 and Maximum value:	1
     * @throws GRBException Handled by client.
     */
    public void setOutputFlag(int flag) throws GRBException {
        this.model.set(GRB.IntParam.OutputFlag, flag);
    }

    /**
     * The MIPFocus parameter allows you to modify your high-level solution strategy, depending on your goals. By default, the Gurobi MIP solver strikes a balance between finding new feasible solutions and proving that the current solution is optimal. If you are more interested in finding feasible solutions quickly, you can select MIPFocus=1. If you believe the solver is having no trouble finding good quality solutions, and wish to focus more attention on proving optimality, select MIPFocus=2. If the best objective bound is moving very slowly (or not at all), you may want to try MIPFocus=3 to focus on the bound.
     *
     * @param focus Minimum value: 0 and Maximum value:	3
     * @throws GRBException Handled by client.
     */
    public void setMIPFocus(int focus) throws GRBException {
        this.model.set(GRB.IntParam.MIPFocus, focus);
    }

    /**
     * Determines how many MIP solutions are stored. For the default value of PoolSearchMode, these are just the solutions that are found along the way in the process of exploring the MIP search tree. For other values of PoolSearchMode, this parameter sets a target for how many solutions to find, so larger values will impact performance.
     *
     * @param solutions Minimum value:	1 and Maximum value:	2000000000
     * @throws GRBException Handled by client.
     */
    public void setPoolSolutions(int solutions) throws GRBException {
        this.model.set(GRB.IntParam.PoolSolutions, solutions);
    }

    /**
     * Selects different modes for exploring the MIP search tree. With the default setting (PoolSearchMode=0), the MIP solver tries to find an optimal solution to the model. It keeps other solutions found along the way, but those are incidental. By setting this parameter to a non-default value, the MIP search will continue after the optimal solution has been found in order to find additional, high-quality solutions. With a setting of 2, it will find the n best solutions, where n is determined by the value of the PoolSolutions parameter. With a setting of 1, it will try to find additional solutions, but with no guarantees about the quality of those solutions. The cost of the solve will increase with increasing values of this parameter.
     *
     * @param mode Minimum value:	0 and Maximum value: 2
     * @throws GRBException
     */
    public void setPoolSearchMode(int mode) throws GRBException {
        this.model.set(GRB.IntParam.PoolSearchMode, mode);
    }

    /**
     * Sets the strategy for handling non-convex quadratic objectives or non-convex quadratic constraints. With setting 0, an error is reported if the original user model contains non-convex quadratic constructs. With setting 1, an error is reported if non-convex quadratic constructs could not be discarded or linearized during presolve. With setting 2, non-convex quadratic problems are solved by means of translating them into bilinear form and applying spatial branching. The default -1 setting is currently equivalent to 1, and may change in future releases to be equivalent to 2.
     * @param nonConvex Minimum value:	-1 and Maximum value: 2
     * @throws GRBException
     */
    public void setNonConvex(int nonConvex) throws GRBException {
        this.model.set(GRB.IntParam.NonConvex, nonConvex);
    }

    public boolean addVariable(GurobiVariable variable) {
        return this.variables.add(variable);
    }

    public boolean removeVariable(GurobiVariable variable) {
        return this.variables.remove(variable);
    }

    public GurobiVariable getVariableByIndex(int i) {
        return this.variables.get(i);
    }

    public void updateModelWithVars() throws GRBException {
        int count = 0;
        for (GurobiVariable v : variables) {
            GRBVar toAddVar = model.addVar(v.getLowerBound(), v.getUpperBound(), 0.0, v.getType(), "X" + count++);
            vars.add(toAddVar);
            v.setGrbVar(toAddVar);
        }
        model.update();
    }

    public boolean addConstraint(GurobiConstraint constraint) {
        return this.constraints.add(constraint);
    }

    public boolean removeConstraint(GurobiConstraint constraint) {
        return this.constraints.remove(constraint);
    }

    public GurobiConstraint getConstraintByIndex(int i) {
        return this.constraints.get(i);
    }

    public GurobiObjConstraint getObjConstraint() {
        return objConstraint;
    }

    public void setObjConstraint(GurobiObjConstraint c) throws GRBException {
        this.objConstraint = c;
        GRBExpr exprLhs;
        if (c.type == ConstraintType.QUADRATIC) {
            exprLhs = new GRBQuadExpr();
            ((GRBQuadExpr) exprLhs).addConstant(c.getLHSConstant());
        } else {
            exprLhs = new GRBLinExpr();
            ((GRBLinExpr) exprLhs).addConstant(c.getLHSConstant());
        }
        for (GurobiVariable v : c.getLeftHandSide().keySet()) {
            if (c.getLhsType() == ConstraintType.QUADRATIC) {
                ((GRBQuadExpr) exprLhs).addTerm(c.getCoeffFromLHSByVar(v), v.getGrbVar());
            } else {
                ((GRBLinExpr) exprLhs).addTerm(c.getCoeffFromLHSByVar(v), v.getGrbVar());
            }
        }
        if (c.getLhsType() == ConstraintType.QUADRATIC)
            for (GurobiVariables vs : c.getLeftHandSideVars().keySet()) {
                ((GRBQuadExpr) exprLhs).addTerm(c.getCoeffFromLHSByVars(vs), vs.getVar().getGrbVar(), vs.getVar2().getGrbVar());
            }


        model.setObjective(exprLhs, c.getGoal());
    }

    public void updateModelWithCons() throws GRBException {
        int count = 0;
        for (GurobiConstraint c : constraints) {

            if (c.type == ConstraintType.LINEAR) {
                GRBLinExpr linExplhs = new GRBLinExpr();
                GRBLinExpr linExprhs = new GRBLinExpr();
                for (GurobiVariable v : c.getLeftHandSide().keySet()) {
                    linExplhs.addTerm(c.getCoeffFromLHSByVar(v), v.getGrbVar());
                }
                linExplhs.addConstant(c.getLHSConstant());
                for (GurobiVariable v : c.getRightHandSide().keySet()) {
                    linExprhs.addTerm(c.getCoeffFromRHSByVar(v), v.getGrbVar());
                }
                linExprhs.addConstant(c.getRHSConstant());
                model.addConstr(linExplhs, c.getSense(), linExprhs, "C" + count++);
            } else {
                GurobiQuadConstraint q = (GurobiQuadConstraint) c;
                GRBExpr exprLhs, exprRhs;
                if (q.getLhsType() == ConstraintType.QUADRATIC) {
                    exprLhs = new GRBQuadExpr();
                    ((GRBQuadExpr) exprLhs).addConstant(q.getLHSConstant());
                } else {
                    exprLhs = new GRBLinExpr();
                    ((GRBLinExpr) exprLhs).addConstant(q.getLHSConstant());
                }
                if (q.getRhsType() == ConstraintType.QUADRATIC) {
                    exprRhs = new GRBQuadExpr();
                    ((GRBQuadExpr) exprRhs).addConstant(q.getRHSConstant());
                } else {
                    exprRhs = new GRBLinExpr();
                    ((GRBLinExpr) exprRhs).addConstant(q.getRHSConstant());
                }
                for (GurobiVariable v : q.getLeftHandSide().keySet()) {
                    if (q.getLhsType() == ConstraintType.QUADRATIC) {
                        ((GRBQuadExpr) exprLhs).addTerm(q.getCoeffFromLHSByVar(v), v.getGrbVar());
                    } else {
                        ((GRBLinExpr) exprLhs).addTerm(q.getCoeffFromLHSByVar(v), v.getGrbVar());
                    }
                }
                for (GurobiVariable v : q.getRightHandSide().keySet()) {
                    if (q.getRhsType() == ConstraintType.QUADRATIC) {
                        ((GRBQuadExpr) exprRhs).addTerm(q.getCoeffFromRHSByVar(v), v.getGrbVar());
                    } else {
                        ((GRBLinExpr) exprRhs).addTerm(q.getCoeffFromRHSByVar(v), v.getGrbVar());
                    }
                }
                if (q.getLhsType() == ConstraintType.QUADRATIC)
                    for (GurobiVariables vs : q.getLeftHandSideVars().keySet()) {
                        ((GRBQuadExpr) exprLhs).addTerm(q.getCoeffFromLHSByVars(vs), vs.getVar().getGrbVar(), vs.getVar2().getGrbVar());
                    }
                if (q.getRhsType() == ConstraintType.QUADRATIC)
                    for (GurobiVariables vs : q.getRightHandSideVars().keySet()) {
                        ((GRBQuadExpr) exprRhs).addTerm(q.getCoeffFromRHSByVars(vs), vs.getVar().getGrbVar(), vs.getVar2().getGrbVar());
                    }
                if (q.getLhsType() == ConstraintType.QUADRATIC && q.getRhsType() == ConstraintType.QUADRATIC)
                    model.addQConstr((GRBQuadExpr) exprLhs, q.getSense(), (GRBQuadExpr) exprRhs, "C" + count++);
                if (q.getLhsType() == ConstraintType.LINEAR && q.getRhsType() == ConstraintType.QUADRATIC)
                    model.addQConstr((GRBLinExpr) exprLhs, q.getSense(), (GRBQuadExpr) exprRhs, "C" + count++);
                if (q.getLhsType() == ConstraintType.QUADRATIC && q.getRhsType() == ConstraintType.LINEAR)
                    model.addQConstr((GRBQuadExpr) exprLhs, q.getSense(), (GRBLinExpr) exprRhs, "C" + count++);

            }
        }
    }

    public void startOptimization() throws GRBException {
        this.model.optimize();
    }

    public void whenInfeasibleWriteOutputTo(String path) throws GRBException {
        if(this.model.get(GRB.IntAttr.Status) == GRB.INFEASIBLE){
            this.model.computeIIS();
            this.model.write(path);
        }
    }
    public GRBEnv getEnv() {
        return env;
    }

    public void setEnv(GRBEnv env) {
        this.env = env;
    }

    public GRBModel getModel() {
        return model;
    }

    public void setModel(GRBModel model) {
        this.model = model;
    }

    public String getId() {
        return id;
    }

    public void setId(String id) {
        this.id = id;
    }

    public ArrayList<GRBVar> getVars() {
        return vars;
    }

    public void setVars(ArrayList<GRBVar> vars) {
        this.vars = vars;
    }

    public ArrayList<GurobiVariable> getVariables() {
        return variables;
    }

    public void setVariables(ArrayList<GurobiVariable> variables) {
        this.variables = variables;
    }

    public ArrayList<GurobiConstraint> getConstraints() {
        return constraints;
    }

    public void setConstraints(ArrayList<GurobiConstraint> constraints) {
        this.constraints = constraints;
    }

    /*
     * ABS
     */
    public void addGenConstraintAbs(GurobiVariable v1, GurobiVariable v2, String name) throws GRBException {
        this.model.addGenConstrAbs(v1.getGrbVar(), v2.getGrbVar(), name);
    }
    /**
     * Min
     */
    public void addGenConstraintMin(GurobiVariable v, GurobiVariable[] vs, Double constant, String name) throws GRBException {
        GRBVar[] vars = new GRBVar[vs.length];
        for (int i = 0; i < vs.length; i++){
            vars[i] = vs[i].getGrbVar();
        }
        this.model.addGenConstrMin(v.getGrbVar(), vars, constant, name);
    }

    /**
     * Max
     */
    public void addGenConstraintMax(GurobiVariable v, GurobiVariable[] vs, Double constant, String name) throws GRBException {
        GRBVar[] vars = new GRBVar[vs.length];
        for (int i = 0; i < vs.length; i++){
            vars[i] = vs[i].getGrbVar();
        }
        this.model.addGenConstrMax(v.getGrbVar(), vars, constant, name);
    }



}
