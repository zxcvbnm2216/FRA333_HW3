# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.ปวริศร์_6536
2.ภาสวร_6548
'''
#=============================================<คำตอบข้อ 1>======================================================#
#code here
import numpy as np
from HW3_utils import FKHW3

# Function: endEffectorJacobianHW3
# Input: q (list of 3 joint angles)
# Output: J_e (Jacobian matrix as a numpy array of shape (6, 3))
def endEffectorJacobianHW3(q):
    # Get forward kinematics data
    R, P, R_e, p_e = FKHW3(q)  # Adjust to match the returned values from HW3_utils

    # Initialize the Jacobian matrix (6x3)
    J_e = np.zeros((6, 3))

    # Joint axes (assuming Z-axis for revolute joints)
    Z0 = np.array([0, 0, 1])  # Base joint axis
    Z1 = R[:, 2, 0]           # Axis of the second joint
    Z2 = R[:, 2, 1]           # Axis of the third joint

    # Joint positions
    P0 = np.array([0, 0, 0])  # Base position
    P1 = P[:, 0]              # Position of the second joint
    P2 = P[:, 1]              # Position of the third joint

    # Compute Jacobian columns
    Jv1 = np.cross(Z0, (p_e - P0))  # Linear velocity due to q1
    Jv2 = np.cross(Z1, (p_e - P1))  # Linear velocity due to q2
    Jv3 = np.cross(Z2, (p_e - P2))  # Linear velocity due to q3

    # Rotational velocities
    Jw1 = Z0  # Rotation due to q1
    Jw2 = Z1  # Rotation due to q2
    Jw3 = Z2  # Rotation due to q3

    # Combine into Jacobian matrix
    J_e[:3, 0] = Jv1
    J_e[:3, 1] = Jv2
    J_e[:3, 2] = Jv3
    J_e[3:, 0] = Jw1
    J_e[3:, 1] = Jw2
    J_e[3:, 2] = Jw3

    return J_e

# Function: checkSingularityHW3
# Input: q (list of 3 joint angles)
# Output: flag (1 if near singularity, 0 if not)
def checkSingularityHW3(q):
    # Get the Jacobian matrix
    J_e = endEffectorJacobianHW3(q)

    # We check for singularity in the 3x3 linear velocity part of the Jacobian (first 3 rows)
    Jv = J_e[:3, :3]

    # Compute determinant
    det_Jv = np.linalg.det(Jv)

    # Define epsilon threshold for singularity
    epsilon = 0.001

    # If determinant is close to zero, it's a singularity
    if abs(det_Jv) < epsilon:
        return 1
    else:
        return 0

# Function: computeEffortHW3
# Input: q (list of 3 joint angles), w (list of 6 external wrench components)
# Output: tau (list of 3 joint torques)
def computeEffortHW3(q, w):
    # Get the Jacobian matrix
    J_e = endEffectorJacobianHW3(q)

    # Convert wrench and Jacobian to numpy arrays
    w = np.array(w)
    J_e = np.array(J_e)

    # Compute joint torques using the transpose of the Jacobian
    tau = np.dot(J_e.T, w)

    return tau.tolist()