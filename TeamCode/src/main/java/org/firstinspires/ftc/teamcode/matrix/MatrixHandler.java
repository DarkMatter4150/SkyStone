package org.firstinspires.ftc.teamcode.matrix;

public class MatrixHandler {

    Matrix a;
    Matrix b;

    public MatrixHandler(Matrix a, Matrix b) {
        this.a = a;
        this.b = b;
    }

    public Matrix addMatrices() {
        if (a.getSize()[0] == b.getSize()[0] && a.getSize()[1] == b.getSize()[1]) {
            Matrix out = new Matrix(a.getSize()[0], a.getSize()[1]);
            for (int i = 0; i < a.getSize()[0]; i++) {
                for (int j = 0; j < a.getSize()[1]; j++) {
                    float val = a.getValue(i, j) + b.getValue(i, j);
                    out.setValue(i, j, val);
                }
            }
            return out;
        } else {
            return null;
        }
    }

    public Matrix subtractMatrices() {
        if (a.getSize()[0] == b.getSize()[0] && a.getSize()[1] == b.getSize()[1]) {
            Matrix out = new Matrix(a.getSize()[0], a.getSize()[1]);
            for (int i = 0; i < a.getSize()[0]; i++) {
                for (int j = 0; j < a.getSize()[1]; j++) {
                    float val = a.getValue(i, j) - b.getValue(i, j);
                    out.setValue(i, j, val);
                }
            }
            return out;
        } else {
            return null;
        }
    }
}
