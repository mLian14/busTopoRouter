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

import gurobi.GRB;

public class GurobiObjConstraint extends GurobiQuadConstraint {

    private int goal;

    public GurobiObjConstraint() {
        this.type = ConstraintType.LINEAR;
    }

    public GurobiObjConstraint(int goal) {
        this.goal = goal;
    }

    public int getGoal() {
        return goal;
    }

    /**
     * @param goal Should be {@link GRB}.MAXIMIZE, {@link GRB}.MINIMIZE Or '1', '-1'
     */
    public void setGoal(int goal) {
        this.goal = goal;
    }

    @Override
    public String toString() {
        String output = "";
        for (GurobiVariables vs : this.leftHandSideVars.keySet()) {
            output += "(" + this.getCoeffFromLHSByVars(vs) + "∙" + vs.getVar().getName() + "∙" + vs.getVar2().getName() + ") + ";
        }
        for (GurobiVariable v : this.leftHandSide.keySet()) {
            output += "(" + this.getCoeffFromLHSByVar(v) + "∙" + v.getName() + ") + ";
        }
        output += "(" + this.getLHSConstant() + ")";


        return "GurobiObjConstraint{Expression=\"" +
                output +
                "\", goal=\"" + (goal == GRB.MAXIMIZE?"MAXIMIZE":"MINIMIZE") +
                "\"}";
    }
}
