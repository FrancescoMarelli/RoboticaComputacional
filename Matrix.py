import numpy as np

# Multiply using method

x = [[1,2,3],[4,5,6]]
y = [[1,2],[3,4],[5,6]]

def matrix_multiply(x,y):
    result = [[0,0],[0,0]]
    for i in range(len(x)):
        for j in range(len(y[0])):
            for k in range(len(y)):
                result[i][j] += x[i][k] * y[k][j]
    return result

def print_matrix(matrix):
    for i in matrix:
        print(i)

print("Matrix using functions: ")
print_matrix(matrix_multiply(x,y))


# Multiply using numpy

x = np.array([[1,2,3],[4,5,6]])
y = np.array([[1,2],[3,4],[5,6]])

print(" ")
print("Matrix using numpy: ")
print(np.dot(x,y))