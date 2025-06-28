# Source Code Overview

This directory contains the core source files for the optimization and robot control components of the project.

---

## 🔧 optimizer_util.cpp

Implements general-purpose optimization utilities, including:

- **BackTrackingLineSearch**:
    - Class implementing the Armijo-rule-based backtracking line search strategy.
    - Validates step size parameters (`alpha`, `beta`).
    - Computes step length ensuring sufficient descent.

- **finite_difference_gradient**:
    - Utility function to numerically approximate gradients via forward finite differences.
    - Input: scalar function \( f(x) \), current point \( x \).
    - Output: approximate gradient \( \nabla f(x) \).

We plan to implement as many functions / classes as possible from [Numerical Optimization](https://link.springer.com/book/10.1007/978-0-387-40065-5) book by Jorge Nocedal and Stephen J. Wright.

---

## 🤖 robot.cpp

Implements a modular robot abstraction using object-oriented design:

### **Classes**
- `SingleArm`
    - Represents a robot with a single arm.
    - Initializes a list of joints (`JointSpec`) with hardcoded revolute joint specifications (todo: YAML support).
    - Placeholder for future integration with sensors and controllers.
    - Implements `sendCommands()` to simulate sending commands to joints.

- `TwoArm`
    - Composed of two `SingleArm` instances (`left_arm`, `right_arm`).
    - Aggregates all joints into a unified `m_joints` list.
    - Can be extended to represent a dual-arm robot system (e.g., bimanual manipulators).

---

## 🔩 robot_joint.cpp

Defines specific joint types and their transformations in 3D space:

### **Joint Types**
- `RevoluteJoint`
    - Computes 4x4 transform using an axis-angle rotation about `m_axis`, then applies `m_offset`.
    - Implements `print()` for human-readable joint info.

- `PrismaticJoint`
    - Computes transform by translating along `m_axis`, scaled by joint position `q`.
    - Also implements a similar `print()` method.

### **JointFactory**
- `JointFactory::createJoint`
    - Factory function to dynamically construct joint instances (`RevoluteJoint`, `PrismaticJoint`) from type name strings.
    - Throws an exception if joint type is unknown.
    - Currently a **bug**: `"prismatic"` type still constructs a `RevoluteJoint`. Should be:
      ```cpp
      else if (type == "prismatic") {
          return std::make_shared<PrismaticJoint>(...);
      }
      ```

---
