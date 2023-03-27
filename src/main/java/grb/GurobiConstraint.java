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

import java.util.HashMap;
import java.util.Map;

public class GurobiConstraint {

    protected char sense;


    protected Map<GurobiVariable, Double> leftHandSide;
    protected Map<GurobiVariable, Double> rightHandSide;

    protected double lhsConstant;
    protected double rhsConstant;


    public ConstraintType type = ConstraintType.LINEAR;

    public GurobiConstraint() {
        this.leftHandSide = new HashMap<>();
        this.rightHandSide = new HashMap<>();
        this.lhsConstant = this.rhsConstant = 0.0;
    }


    public double getLHSConstant() {
        return lhsConstant;
    }

    public void setLHSConstant(double lhsConstant) {
        this.lhsConstant = lhsConstant;
    }

    public double getRHSConstant() {
        return rhsConstant;
    }

    public void setRHSConstant(double rhsConstant) {
        this.rhsConstant = rhsConstant;
    }


    public Map<GurobiVariable, Double> getLeftHandSide() {
        return leftHandSide;
    }

    public void setLeftHandSide(Map<GurobiVariable, Double> leftHandSide) {
        this.leftHandSide = leftHandSide;
    }

    public Map<GurobiVariable, Double> getRightHandSide() {
        return rightHandSide;
    }

    public void setRightHandSide(Map<GurobiVariable, Double> rightHandSide) {
        this.rightHandSide = rightHandSide;
    }

    public void addToLHS(GurobiVariable v, double coefficient) {
        this.leftHandSide.put(v, coefficient);
    }

    public void addToRHS(GurobiVariable v, double coefficient) {
        this.rightHandSide.put(v, coefficient);
    }

    public double deleteFromLHS(GurobiVariable v) {
        return this.leftHandSide.remove(v);
    }

    public double deleteFromRHS(GurobiVariable v) {
        return this.rightHandSide.remove(v);
    }

    public double getCoeffFromLHSByVar(GurobiVariable v) {
        return this.leftHandSide.get(v);
    }

    public double getCoeffFromRHSByVar(GurobiVariable v) {
        return this.rightHandSide.get(v);
    }

    public char getSense() {
        return sense;
    }

    /**
     * @param sense Should be {@link GRB}.GREATER_EQUAL, {@link GRB}.LESS_EQUAL or {@link GRB}.EQUAL Or '>', '<' or '='.
     */
    public void setSense(char sense) {
        this.sense = sense;
    }

    @Override
    public String toString() {
        String output = "";

        for (GurobiVariable v : this.leftHandSide.keySet()) {
            output += "(" + this.getCoeffFromLHSByVar(v) + "∙" + v.getName() + ") + ";
        }
        output += "(" +this.getLHSConstant() + ") " +  (this.getSense() == '=' ? "= ": this.getSense() == '>'? "≥ ": "≤ ");
        for (GurobiVariable v : this.rightHandSide.keySet()) {
            output += "(" + this.getCoeffFromRHSByVar(v) + "∙" + v.getName() + ") + ";
        }
        output += "(" + this.getRHSConstant() + ")";


        return "GurobiConstraint{{Expression=\"" +
                output +
                "\", type=" + type +
                '}';
    }
}
