package org.firstinspires.ftc.teamcode.matrix;

public class Matrix {

    float[][] matrix;

    public Matrix(int x, int y) {
        matrix = new float[y][x];
    }

    @Deprecated
    public void setValue(int xId, int yId, float val) {
        matrix[yId][xId] = val;
    }

    public void setValues(float[] elements) {
        if (elements.length == matrix.length * matrix[0].length) {
            int k = 0;
            for (int i = 0; i < matrix.length; i++) {
                for (int j = 0; j < matrix[0].length; j++) {
                    matrix[i][j] = elements[k];
                    k++;
                }
            }
        }
    }

    public void scalar(float scalar) {
        float[][] temp = matrix;
        matrix[0][0] = temp[0][0] * scalar;
        matrix[0][1] = temp[0][1] * scalar;
        matrix[1][0] = temp[1][0] * scalar;
        matrix[1][1] = temp[1][1] * scalar;
    }

    public void setMotorValues(float n, boolean isXArray) {
        if (matrix.length == 2 && matrix[0].length == 2) {
            if (isXArray) {
                //which is more efficient?
                //this?
                matrix[0][0] = n;
                matrix[0][1] = -n;
                matrix[1][0] = -n;
                matrix[1][1] = n;
                //or this?
//                matrix = new float[][]{new float[]{n, -n}, new float[]{-n, n}};
            } else {
//                matrix = new float[][]{new float[]{n, n}, new float[]{n, n}};
            }
        }
    }

    public float getValue(int xId, int yId) {
        return matrix[yId][xId];
    }

    public int[] getSize() {
        int[] x = new int[2];
        x[0] = matrix.length;
        x[1] = matrix[0].length;
        return x;
    }
}
