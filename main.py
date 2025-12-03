import pybullet as p
import pybullet_data
import time
import math
import random

# Distância em 2D (X, Y)
def distance2d(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

# Move o drone em direção a um ponto (X, Y) com velocidade fixa
def move_towards_xy(drone_id, target_xy, speed=1.0):
    pos, _ = p.getBasePositionAndOrientation(drone_id)
    x, y, _ = pos

    dx = target_xy[0] - x
    dy = target_xy[1] - y
    dist = math.sqrt(dx*dx + dy*dy)

    # Se está perto, chegou
    if dist < 0.3:  # tolerância maior
        p.resetBaseVelocity(drone_id, [0, 0, 0])
        return True

    vx = speed * dx / dist
    vy = speed * dy / dist
    p.resetBaseVelocity(drone_id, [vx, vy, 0])
    return False

def generate_random_targets(n, min_c=-5, max_c=5):
    targets = []
    for _ in range(n):
        x = random.uniform(min_c, max_c)
        y = random.uniform(min_c, max_c)
        z = 1
        targets.append([x, y, z])
    return targets


def main():
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, 0)
    p.setTimeStep(1.0/120.0)

    p.loadURDF("plane.urdf")

    # Drone nasce LONGE dos alvos
    base_pos = [0, -8, 1]

    drone_id = p.createMultiBody(
        baseMass=1,
        baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1,0.1,0.03]),
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1,0.1,0.03], rgbaColor=[0,1,0,1]),
        basePosition=base_pos,
        baseOrientation=p.getQuaternionFromEuler([0,0,0])
    )

    # impede sleep
    p.changeDynamics(drone_id, -1, activationState=1)

    # leve impulso inicial
    p.resetBaseVelocity(drone_id, [0.2, 0.1, 0])

    # Criar alvos aleatórios
    NUM_TARGETS = 6
    positions = generate_random_targets(NUM_TARGETS)

    target_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.18, rgbaColor=[1,0,0,1])
    target_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.18)

    targets = []
    for i, pos in enumerate(positions):
        bid = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=target_collision,
            baseVisualShapeIndex=target_visual,
            basePosition=pos
        )
        targets.append({
            "name": f"T{i+1}",
            "pos": pos,
            "body_id": bid,
            "detected": False,
            "visited": False,
        })

    print("\n[ALVOS GERADOS]:")
    for t in targets:
        print(" ", t["name"], t["pos"])

    remaining = targets.copy()
    sensor_range = 4.0
    speed = 1.2

    print("\n🟢 Iniciando visita aos pontos...")

    # LOOP PRINCIPAL
    while p.isConnected() and len(remaining) > 0:

        pos, _ = p.getBasePositionAndOrientation(drone_id)
        drone_xy = [pos[0], pos[1]]

        # escolhe mais próximo
        current = min(remaining, key=lambda t: distance2d(drone_xy, t["pos"]))

        target_xy = [current["pos"][0], current["pos"][1]]

        reached = False

        while p.isConnected() and not reached:

            pos, _ = p.getBasePositionAndOrientation(drone_id)
            drone_xy = [pos[0], pos[1]]

            # detectar
            if not current["detected"]:
                if distance2d(drone_xy, target_xy) <= sensor_range:
                    current["detected"] = True
                    print(f"[SENSOR] Detectei {current['name']} em {current['pos']}")

            # mover
            reached = move_towards_xy(drone_id, target_xy, speed)

            # chegou
            if reached:
                current["visited"] = True
                print(f"[VISITA] Cheguei em {current['name']} {current['pos']}")
                remaining = [t for t in remaining if t is not current]
                time.sleep(0.4)

            p.stepSimulation()
            time.sleep(1.0/120.0)

    print("\n🟢 Finalizado! Todos os pontos foram visitados:")
    for t in targets:
        print(f"  {t['name']}  detectado={t['detected']} visitado={t['visited']}")

    input("\nPressione ENTER para fechar...")
    p.disconnect()


if __name__ == "__main__":
    main()
