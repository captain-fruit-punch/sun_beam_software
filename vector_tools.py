import numpy as np

def calculate_angle_between_vectors(vector1, vector2):
    dot_product = np.dot(vector1, vector2)
    magnitude_product = np.linalg.norm(vector1) * np.linalg.norm(vector2)
    return np.rad2deg(np.arccos(dot_product / magnitude_product))

def calculate_magnitude_of_vector(vector):
    return np.linalg.norm(vector)

def calculate_unit_vector(vector):
    return vector / np.linalg.norm(vector)

def calculate_cross_product(vector1, vector2):
    return np.cross(vector1, vector2)

def calculate_dot_product(vector1, vector2):
    return np.dot(vector1, vector2)

def calculate_projection(vector1, vector2):
    return np.dot(vector1, vector2) / np.linalg.norm(vector2)

def rotate_vector(vector, angle):
    angle_rad = np.deg2rad(angle)
    rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)], [np.sin(angle_rad), np.cos(angle_rad)]])
    return np.dot(rotation_matrix, vector)

def __main__():
    vector1 = np.array([1, 0])
    vector2 = np.array([0, 1])
    print("calculate_angle_between_vectors(vector1, vector2): ", calculate_angle_between_vectors(vector1, vector2))
    print("calculate_magnitude_of_vector(vector1): ", calculate_magnitude_of_vector(vector1))
    print("calculate_unit_vector(vector1): ", calculate_unit_vector(vector1))
    print("calculate_cross_product(vector1, vector2): ", calculate_cross_product(vector1, vector2))
    print("calculate_dot_product(vector1, vector2): ", calculate_dot_product(vector1, vector2))
    print("calculate_projection(vector1, vector2): ", calculate_projection(vector1, vector2))
    print("rotate_vector(vector1, 90): ", rotate_vector(vector1, 90))


if __name__ == "__main__":
    __main__()
