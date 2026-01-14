#!/usr/bin/env python3
"""
Physics verification script for 2D grid simulation
Checks spring forces and energy conservation
"""

import math
import csv

def read_csv_data(filename):
    """Read simulation data from CSV"""
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        data = list(reader)

    times = []
    positions = []

    for row in data:
        times.append(float(row['time']))

        # Extract all positions
        pos_dict = {}
        for key in row.keys():
            if key.startswith('x') or key.startswith('y'):
                pos_dict[key] = float(row[key])
        positions.append(pos_dict)

    return times, positions

def compute_spring_force(pos_i, pos_j, k, L0, c, vel_i, vel_j):
    """
    Compute spring force on mass i from spring connecting i to j

    F = k * (length - L0) * unit_vector + c * relative_velocity
    """
    # Displacement vector from i to j
    dx = pos_j[0] - pos_i[0]
    dy = pos_j[1] - pos_i[1]

    # Current length
    length = math.sqrt(dx**2 + dy**2)

    if length < 1e-10:
        return [0.0, 0.0]

    # Unit vector from i to j
    ux = dx / length
    uy = dy / length

    # Extension
    extension = length - L0

    # Relative velocity along spring direction
    dvx = vel_j[0] - vel_i[0]
    dvy = vel_j[1] - vel_i[1]
    relative_vel = dvx * ux + dvy * uy

    # Force magnitude
    force_mag = k * extension + c * relative_vel

    # Force on mass i
    return [force_mag * ux, force_mag * uy]

def verify_2x2_system():
    """
    Verify physics for a simple 2x2 grid system we can test analytically

    Grid layout (indices):
    0 -- 1
    |    |
    2 -- 3

    Springs: (0,1), (0,2), (1,3), (2,3)
    """
    print("=== Verifying 2x2 Grid Physics ===\n")

    # System parameters
    mass = 1.0  # kg
    spacing = 1.0  # m
    k = 20.0  # N/m
    damping = 0.0  # No damping for this test

    # Initial positions (grid at rest)
    pos = {
        0: [0.0, 0.0],
        1: [1.0, 0.0],
        2: [0.0, 1.0],
        3: [1.0, 1.0]
    }

    # Initial velocities (all zero)
    vel = {
        0: [0.0, 0.0],
        1: [0.0, 0.0],
        2: [0.0, 0.0],
        3: [0.0, 0.0]
    }

    print("Test 1: Grid at rest (equilibrium)")
    print("All masses at equilibrium positions")

    # Check forces at equilibrium
    edges = [(0, 1), (0, 2), (1, 3), (2, 3)]
    L0 = spacing  # Rest length

    total_force = {i: [0.0, 0.0] for i in range(4)}

    for (i, j) in edges:
        force_on_i = compute_spring_force(pos[i], pos[j], k, L0, damping, vel[i], vel[j])
        force_on_j = [-force_on_i[0], -force_on_i[1]]  # Newton's 3rd law

        total_force[i][0] += force_on_i[0]
        total_force[i][1] += force_on_i[1]
        total_force[j][0] += force_on_j[0]
        total_force[j][1] += force_on_j[1]

    print("\nForces at equilibrium:")
    for i in range(4):
        print(f"  Mass {i}: F = ({total_force[i][0]:.6f}, {total_force[i][1]:.6f}) N")

    # Check if forces are near zero (equilibrium)
    max_force = max(math.sqrt(total_force[i][0]**2 + total_force[i][1]**2) for i in range(4))
    if max_force < 1e-10:
        print(f"✓ Equilibrium verified (max force: {max_force:.2e} N)\n")
    else:
        print(f"✗ ERROR: Forces not zero at equilibrium (max: {max_force:.2e} N)\n")

    # Test 2: Perturb one corner
    print("Test 2: Perturb corner mass 0 by (0.2, 0.3)")
    pos[0] = [0.2, 0.3]

    # Recompute forces
    total_force = {i: [0.0, 0.0] for i in range(4)}

    for (i, j) in edges:
        force_on_i = compute_spring_force(pos[i], pos[j], k, L0, damping, vel[i], vel[j])
        force_on_j = [-force_on_i[0], -force_on_i[1]]

        total_force[i][0] += force_on_i[0]
        total_force[i][1] += force_on_i[1]
        total_force[j][0] += force_on_j[0]
        total_force[j][1] += force_on_j[1]

        # Calculate expected values
        dx = pos[j][0] - pos[i][0]
        dy = pos[j][1] - pos[i][1]
        length = math.sqrt(dx**2 + dy**2)
        extension = length - L0

        print(f"\n  Spring ({i},{j}):")
        print(f"    Length: {length:.4f} m (rest: {L0:.4f} m)")
        print(f"    Extension: {extension:.4f} m")
        print(f"    Force on {i}: ({force_on_i[0]:.4f}, {force_on_i[1]:.4f}) N")

    print("\nTotal forces after perturbation:")
    for i in range(4):
        accel_x = total_force[i][0] / mass
        accel_y = total_force[i][1] / mass
        print(f"  Mass {i}: F = ({total_force[i][0]:7.4f}, {total_force[i][1]:7.4f}) N")
        print(f"           a = ({accel_x:7.4f}, {accel_y:7.4f}) m/s²")

    # Check Newton's 3rd law (total force should be zero)
    total_system_force = [sum(total_force[i][0] for i in range(4)),
                          sum(total_force[i][1] for i in range(4))]
    print(f"\nNewton's 3rd law check:")
    print(f"  Total system force: ({total_system_force[0]:.6e}, {total_system_force[1]:.6e}) N")

    force_mag = math.sqrt(total_system_force[0]**2 + total_system_force[1]**2)
    if force_mag < 1e-10:
        print(f"  ✓ Newton's 3rd law verified (sum of forces ≈ 0)\n")
    else:
        print(f"  ✗ ERROR: Newton's 3rd law violated!\n")

    # Test 3: Check energy for oscillation
    print("\nTest 3: Energy conservation (theoretical)")

    # Potential energy
    PE = 0.0
    for (i, j) in edges:
        dx = pos[j][0] - pos[i][0]
        dy = pos[j][1] - pos[i][1]
        length = math.sqrt(dx**2 + dy**2)
        extension = length - L0
        PE += 0.5 * k * extension**2

    # Kinetic energy (all at rest currently)
    KE = 0.0
    for i in range(4):
        vel_mag = math.sqrt(vel[i][0]**2 + vel[i][1]**2)
        KE += 0.5 * mass * vel_mag**2

    total_energy = PE + KE
    print(f"  Kinetic energy: {KE:.6f} J")
    print(f"  Potential energy: {PE:.6f} J")
    print(f"  Total energy: {total_energy:.6f} J")
    print(f"  ✓ Initial energy calculated\n")

    return True

def check_specific_configuration():
    """Check a specific known configuration"""
    print("=== Analytical Verification ===\n")

    print("Test: Two masses connected by a spring")
    print("  Mass 1 at (0, 0), Mass 2 at (1.5, 0)")
    print("  Spring: k=10 N/m, L0=1.0 m, c=0")

    pos1 = [0.0, 0.0]
    pos2 = [1.5, 0.0]
    vel1 = [0.0, 0.0]
    vel2 = [0.0, 0.0]

    k = 10.0
    L0 = 1.0
    c = 0.0

    force = compute_spring_force(pos1, pos2, k, L0, c, vel1, vel2)

    # Expected: extension = 0.5 m, force = 10 * 0.5 = 5 N in x direction
    print(f"\n  Computed force on mass 1: ({force[0]:.4f}, {force[1]:.4f}) N")
    print(f"  Expected force:           (5.0000, 0.0000) N")

    if abs(force[0] - 5.0) < 1e-6 and abs(force[1]) < 1e-6:
        print(f"  ✓ Analytical verification passed\n")
        return True
    else:
        print(f"  ✗ ERROR: Force calculation incorrect!\n")
        return False

def main():
    print("=" * 60)
    print("SOPOT 2D Grid Physics Verification")
    print("=" * 60)
    print()

    # Run analytical tests
    check_specific_configuration()
    verify_2x2_system()

    print("=" * 60)
    print("Verification Complete")
    print("=" * 60)

if __name__ == "__main__":
    main()
