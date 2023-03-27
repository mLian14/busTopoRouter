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
import gurobi.GRBException;
import gurobi.GRBVar;

import java.util.ArrayList;
import java.util.Objects;

public class GurobiVariable {
    private char type;
    private double lowerBound, upperBound;
    private ArrayList<GurobiConstraint> constrains;
    private double attribute;
    private String name;
    private GRBVar grbVar;


    public GurobiVariable(char type, double lowerBound, double upperBound, String name) {
        this.type = type;
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
        this.name = name;
    }


    /**
     * @param type Should be {@link GRB}.CONTINUOUS, {@link GRB}.BINARY or {@link GRB}.INTEGER. Or 'C', 'B' or 'I'.
     */

    public GurobiVariable(char type) {
        this.type = type;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public GRBVar getGrbVar() {
        return grbVar;
    }

    public void setGrbVar(GRBVar grbVar) {
        this.grbVar = grbVar;
    }

    public char getType() {
        return type;
    }

    public void setType(char type) {
        this.type = type;
    }

    public double getLowerBound() {
        return lowerBound;
    }

    public void setLowerBound(double lowerBound) {
        this.lowerBound = lowerBound;
    }

    public double getUpperBound() {
        return upperBound;
    }

    public void setUpperBound(double upperBound) {
        this.upperBound = upperBound;
    }

    public int getIntResult() throws GRBException {
        int rt = 0;
        double d_rt = this.grbVar.get(GRB.DoubleAttr.X);
        rt = Math.toIntExact(Math.round(d_rt));


        return rt;

    }

    public double getContResult() throws GRBException {
        return this.grbVar.get(GRB.DoubleAttr.X);
    }

    public double getAttribute() {
        return attribute;
    }

    public void setAttribute(double attribute) {
        this.attribute = attribute;
    }

    public ArrayList<GurobiConstraint> getConstrains() {
        return constrains;
    }

    public void setConstrains(ArrayList<GurobiConstraint> constrains) {
        this.constrains = constrains;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        GurobiVariable that = (GurobiVariable) o;
        return Objects.equals(grbVar, that.grbVar);
    }

    @Override
    public int hashCode() {
        return Objects.hash(grbVar);
    }

    @Override
    public String toString() {
        return "GurobiVariable{" +
                "type=" + type +
                ", lowerBound=" + lowerBound +
                ", upperBound=" + upperBound +
                ", attribute=" + attribute +
                ", name='" + name + '\'' +
                '}';
    }
}
