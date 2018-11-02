import math

class Vector4:
    def __init__(self, x, y, z, w):
        self.detail = [x, y, z, w]

class TMatrix:

    def __init__(self, a00: float = 0, a01: float = 0, a02: float = 0, a03: float = 0, a10: float = 0, a11: float = 0, a12: float = 0, a13: float = 0, a20: float = 0, a21: float = 0, a22: float = 0, a23: float = 0, a30: float = 0, a31: float = 0, a32: float = 0, a33: float = 0):
        self.detail = [[a00, a01, a02, a03], [a10, a11, a12, a13], [a20, a21, a22, a23], [a30, a31, a32, a33]]

    def mult(self, other_matrix):
        result = TMatrix()

        # formula: https://en.wikipedia.org/wiki/Matrix_multiplication_algorithm
        # this one is the reverse version for column-major

        # Assume A [n x m] and B [m x p] -> C [n x p]
        # for i from 1 to n (rows of C)
        #   for j from 1 to p (columns of C)
        #       for k from 1 to m (is columns of A equal rows of B also)
        #
        #           C[i][j] += A[i][k] * B[k][j] (for row-major)
        #           or
        #           C[j][i] += A[k][i] * B[j][k] (for column-major)

        for i in range(len(self.detail[0])):                   # each index in each row in first matrix

            for j in range(len(other_matrix.detail)):          # each index in each column in second matrix

                for k in range(len(self.detail)):              # each column in each column in first matrix (= rows in second matrix )

                    result.detail[j][i] += self.detail[k][i] * other_matrix.detail[j][k]

        return result


    def mult_vec(self, vector):
        result = Vector4(0, 0, 0, 0)

        for i in range(len(self.detail)):
            for j in range(len(vector.detail)):
                result.detail[i] += self.detail[j][i] * vector.detail[j]

        return result

def make_trans_mat(x, y, z):
    return TMatrix(1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1)


def make_rot_mat(degree, axis):
    rad = math.radians(degree)
    cos = math.cos(rad)
    sin = math.sin(rad)

    if axis == "x":
        return TMatrix(1, 0, 0, 0, 0, cos, sin, 0, 0, -sin, cos, 0, 0, 0, 0, 1)
    elif axis == "y":
        return TMatrix(cos, 0, -sin, 0, 0, 1, 0, 0, sin, 0, cos, 0, 0, 0, 0, 1)
    else:
        return TMatrix(cos, sin, 0, 0, -sin, cos, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)

def make_scale_mat(sx, sy, sz):
    return TMatrix(sx, 0, 0, 0, 0, sy, 0, 0, 0, 0, sz, 0, 0, 0, 0, 1)

# https://en.wikipedia.org/wiki/Euclidean_distance
def euclidean_distance(point1: Vector4, point2: Vector4):
    return math.sqrt(math.pow((point1.detail[0] - point2.detail[0]), 2) + math.pow((point1.detail[1] - point2.detail[1]), 2) + math.pow((point1.detail[2] - point2.detail[2]), 2))


def main():
    matA = TMatrix(1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15, 4, 8, 12, 16)
    matB = TMatrix(1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15, 8, 16)
    print("1.1: ")
    print("A: ")
    print(matA.detail)
    print("B: ")
    print(matB.detail)
    print("A . B: ")
    print(matA.mult(matB).detail)
    print()

    # 1.2
    print("1.2:")
    trans = make_trans_mat(1, 2, 3)
    print(trans.detail)
    rot1 = make_rot_mat(45, 'x')
    print(rot1.detail)
    rot2 = make_rot_mat(90, 'y')
    print(rot2.detail)
    rot3 = make_rot_mat(120, 'z')
    print(rot3.detail)
    scale = make_scale_mat(1, 2, 3)
    print(scale.detail)
    print()

    # 1.3
    print("1.3: ")
    print(euclidean_distance(Vector4(2, 4, 6, 2), Vector4(0, 0, 0, 1)))
    print()

    # 1.4
    vec = matA.mult_vec(Vector4(1, 2, 3, 1))
    print("1.4: ")
    print(vec.detail)
    print()

    # 1.5
    print("1.5: ")
    rot90 = make_rot_mat(90, 'x')
    print(rot90.detail)
    found = False

    # loop from 1 to 360 degrees
    for i in range(0, 361):
        alpha = make_rot_mat(i, 'z')

        for j in range(0, 361):
            if j != i:
                beta = make_rot_mat(j, 'y')
                left = rot90.mult(alpha)
                right = beta.mult(rot90)
                correct = True

                for row in range(len(left.detail[0])):
                    for column in range(len(left.detail)):
                        if left.detail[column][row] != right.detail[column][row]:
                            correct = False
                            break
                    if not correct:
                        break

                found = correct

                if found:
                    break
        if found:
            break

    if found:
        print("answer: " + str(i) + " - " + str(j))
    else:
        print("no pair of angles can satisfy the equation")

main()
