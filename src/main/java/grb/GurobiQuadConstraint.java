/*
 * Copyright (c) 2021. Meng Lian and Yushen Zhang
 *
 *   CONFIDENTIAL
 *   __________________
 *
 *   Meng Lian and Yushen Zhang
 *   All Rights Reserved.
 *
 *   NOTICE:  All information contained herein is, and remains
 *   the property of Meng Lian, Yushen Zhang and the Technical University
 *   of Munich, if applies. The intellectual and technical concepts contained
 *   herein are proprietary to Meng Lian, Yushen Zhang and/or the Technical University
 *   of Munich and may be covered by European and Foreign Patents,
 *   patents in process, and are protected by trade secret or copyright law.
 *   Dissemination of this information or reproduction of this material
 *   is strictly forbidden unless prior written permission is obtained
 *   from Meng Lian and Yushen Zhang.
 */

package grb;

import java.util.HashMap;
import java.util.Map;

public class GurobiQuadConstraint extends GurobiConstraint {
    Map<GurobiVariables, Double> leftHandSideVars;
    Map<GurobiVariables, Double> rightHandSideVars;
    private ConstraintType lhsType = ConstraintType.LINEAR;
    private ConstraintType rhsType = ConstraintType.LINEAR;


    public GurobiQuadConstraint() {
        this.leftHandSideVars = new HashMap<>();
        this.rightHandSideVars = new HashMap<>();
        this.type = ConstraintType.QUADRATIC;
    }

    public void addToLHS(GurobiVariable v1, GurobiVariable v2, double coefficient) {
        this.lhsType = ConstraintType.QUADRATIC;
        this.type = ConstraintType.QUADRATIC;
        this.leftHandSideVars.put(new GurobiVariables(v1, v2), coefficient);
    }

    public void addToRHS(GurobiVariable v1, GurobiVariable v2, double coefficient) {
        this.rhsType = ConstraintType.QUADRATIC;
        this.type = ConstraintType.QUADRATIC;
        this.rightHandSideVars.put(new GurobiVariables(v1, v2), coefficient);
    }

    public double getCoeffFromLHSByVars(GurobiVariable v1, GurobiVariable v2) {
        for (GurobiVariables vs : this.leftHandSideVars.keySet()) {
            if (vs.hasVar(v1) && vs.hasVar(v2)) return this.leftHandSideVars.get(vs);
        }
        return 0.0;
    }

    public double getCoeffFromLHSByVars(GurobiVariables vs) {
        return this.leftHandSideVars.get(vs);
    }

    public double getCoeffFromRHSByVars(GurobiVariable v1, GurobiVariable v2) {
        for (GurobiVariables vs : this.rightHandSideVars.keySet()) {
            if (vs.hasVar(v1) && vs.hasVar(v2)) return this.leftHandSideVars.get(vs);
        }
        return 0.0;
    }

    public double getCoeffFromRHSByVars(GurobiVariables vs) {
        return this.rightHandSideVars.get(vs);
    }

    public Map<GurobiVariables, Double> getLeftHandSideVars() {
        return leftHandSideVars;
    }

    public void setLeftHandSideVars(Map<GurobiVariables, Double> leftHandSideVars) {
        this.leftHandSideVars = leftHandSideVars;
    }

    public Map<GurobiVariables, Double> getRightHandSideVars() {
        return rightHandSideVars;
    }

    public void setRightHandSideVars(Map<GurobiVariables, Double> rightHandSideVars) {
        this.rightHandSideVars = rightHandSideVars;
    }

    public ConstraintType getLhsType() {
        return lhsType;
    }

    public void setLhsType(ConstraintType lhsType) {

        this.lhsType = lhsType;
    }

    public ConstraintType getRhsType() {
        return rhsType;
    }

    public void setRhsType(ConstraintType rhsType) {

        this.rhsType = rhsType;
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
        output +=  "(" + this.getLHSConstant() + ") " +(this.getSense() == '=' ? "= ": this.getSense() == '>'? "≥ ": "≤ ");
        for (GurobiVariables vs : this.rightHandSideVars.keySet()) {
            output += "(" + this.getCoeffFromRHSByVars(vs) + "∙" + vs.getVar().getName() + "∙" + vs.getVar2().getName() + ") + ";
        }
        for (GurobiVariable v : this.rightHandSide.keySet()) {
            output += "(" + this.getCoeffFromRHSByVar(v) + "∙" + v.getName() + ") + ";
        }
        output += "(" + this.getRHSConstant() +")";


        return "GurobiQuadConstraint{Expression=\"" +
                output +
                "\", type=" + type +
                '}';
    }

}
