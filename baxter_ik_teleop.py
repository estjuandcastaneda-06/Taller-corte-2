import os
import sys
import time
import argparse

import numpy as np
import pybullet as p
import pybullet_data

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(HERE, ".."))
from common.serial_bridge import SerialConsole 

ROBOTS_DATA = os.path.join(HERE, "..", "pybullet_robots", "data")
BAXTER_URDF = os.path.join(ROBOTS_DATA, "baxter_common", "baxter_description", "urdf", "toms_baxter.urdf")

SIM_HZ = 240
MOVE_SPEED = 0.35        # m/s de la pinza con el joystick a tope
GRASP_DISTANCE = 0.07    # m, distancia máxima pinza-objeto para poder agarrar
TABLE_TOP_Z = -0.20      # altura de la mesa (el origen del robot está en el torso)
CUBE_START = [0.80, 0.05, TABLE_TOP_Z + 0.025]
DROP_ZONE = [0.55, 0.45]


WS_MIN = np.array([0.35, -0.75, TABLE_TOP_Z + 0.01])
WS_MAX = np.array([1.05, 0.75, 0.60])

ARMS = {
    "izquierdo": {"ee": 48, "prefix": "left_", "fingers": (49, 51)},
    "derecho":   {"ee": 26, "prefix": "right_", "fingers": (27, 29)},
}

REST = {"s0": 0.0, "s1": -0.55, "e0": 0.0, "e1": 1.5, "w0": 0.0, "w1": 0.6, "w2": 0.0}
GRIPPER_DOWN = p.getQuaternionFromEuler([np.pi, 0, 0])


class BaxterTeleop:
    def __init__(self, gui=True):
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / SIM_HZ)

        p.loadURDF("plane.urdf", [0, 0, -0.93])
        if not os.path.exists(BAXTER_URDF):
            raise SystemExit(f"No se encontró el URDF de Baxter en:\n  {BAXTER_URDF}\n"
                             "Clona https://github.com/erwincoumans/pybullet_robots dentro de la "
                             "carpeta del taller (debe quedar Taller-Segundo-Corte/pybullet_robots).")
        self.robot = p.loadURDF(BAXTER_URDF, [0, 0, 0], useFixedBase=True,
                                flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)


        half = [0.35, 0.65, (TABLE_TOP_Z + 0.93) / 2]
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=[0.55, 0.4, 0.3, 1])
        self.table = p.createMultiBody(0, col, vis, [0.75, 0, -0.93 + half[2]])
        zone = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.06, 0.06, 0.001], rgbaColor=[1, 0, 0, 0.6])
        p.createMultiBody(0, -1, zone, [DROP_ZONE[0], DROP_ZONE[1], TABLE_TOP_Z + 0.001])
        self.cube = p.loadURDF("cube_small.urdf", CUBE_START, globalScaling=0.8)
        p.changeVisualShape(self.cube, -1, rgbaColor=[0.1, 0.8, 0.1, 1])
        p.changeDynamics(self.cube, -1, lateralFriction=1.0)


        self.joints = [i for i in range(p.getNumJoints(self.robot))
                       if p.getJointInfo(self.robot, i)[3] > -1]
        self.names = [p.getJointInfo(self.robot, i)[1].decode() for i in self.joints]
        self.ll = [p.getJointInfo(self.robot, i)[8] for i in self.joints]
        self.ul = [p.getJointInfo(self.robot, i)[9] for i in self.joints]
        self.jr = [u - l for l, u in zip(self.ll, self.ul)]
        self.rest = []
        for n in self.names:
            key = n.split("_")[-1]
            if n.startswith(("left_", "right_")) and key in REST:
                val = REST[key]

                if n.startswith("right_") and key in ("s0", "e0", "w0", "w2"):
                    val = -val
                self.rest.append(val)
            else:
                self.rest.append(0.0)
        for j, q in zip(self.joints, self.rest):
            p.resetJointState(self.robot, j, q)
        self.cmd = list(self.rest)  

        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.resetDebugVisualizerCamera(2.2, 60, -25, [0.5, 0, 0.0])

  
        self.targets = {"izquierdo": np.array([0.65, 0.30, 0.05]),
                        "derecho": np.array([0.65, -0.30, 0.05])}
        self.grip_closed = {"izquierdo": False, "derecho": False}
        self.arm = "izquierdo"
        self.grasp_cid = None
        self.grasp_arm = None


        for arm in ARMS:
            self._solve_ik(arm, iterations=40, teleport=True)
        for j, q in zip(self.joints, self.cmd):
            p.resetJointState(self.robot, j, q)
        self._set_fingers_all()

        self.txt_target = p.addUserDebugText("TARGET", self.targets[self.arm].tolist(), [1, 0, 0], 1.5)
        self.txt_status = p.addUserDebugText("", [0.2, 0, 0.95], [0, 0, 0], 1.3)
        self._update_text(force=True)

 
    def _solve_ik(self, arm, iterations=1, teleport=False):
        info = ARMS[arm]
        prefix = info["prefix"]
        target = self.targets[arm].tolist()
        for _ in range(iterations):
            q = p.calculateInverseKinematics(
                self.robot, info["ee"], target, GRIPPER_DOWN,
                lowerLimits=self.ll, upperLimits=self.ul, jointRanges=self.jr,
                restPoses=self.rest, maxNumIterations=100, residualThreshold=1e-4)
            for k, n in enumerate(self.names):
                if n.startswith(prefix):  
                    self.cmd[k] = q[k]
                    if teleport:
                        p.resetJointState(self.robot, self.joints[k], q[k])

    def _apply_motors(self):
        for k, j in enumerate(self.joints):
            n = self.names[k]
            if "finger" in n:
                continue
            p.setJointMotorControl2(self.robot, j, p.POSITION_CONTROL, targetPosition=self.cmd[k],
                                    force=200, maxVelocity=1.5, positionGain=0.3)

    def _set_fingers_all(self):
        for arm, info in ARMS.items():
            opening = 0.0 if self.grip_closed[arm] else 0.02
            lf, rf = info["fingers"]
            p.setJointMotorControl2(self.robot, lf, p.POSITION_CONTROL, targetPosition=opening, force=20)
            p.setJointMotorControl2(self.robot, rf, p.POSITION_CONTROL, targetPosition=-opening, force=20)


    def ee_pos(self, arm=None):
        return np.array(p.getLinkState(self.robot, ARMS[arm or self.arm]["ee"])[4])

    def toggle_grip(self):
        arm = self.arm
        if self.grasp_cid is not None and self.grasp_arm == arm:
            p.removeConstraint(self.grasp_cid)
            self.grasp_cid = None
            self.grasp_arm = None
            self.grip_closed[arm] = False
            print("Pinza abierta: objeto liberado.")
        elif not self.grip_closed[arm]:
            self.grip_closed[arm] = True
            cube_pos, cube_orn = p.getBasePositionAndOrientation(self.cube)
            dist = np.linalg.norm(self.ee_pos() - np.array(cube_pos))
            if dist < GRASP_DISTANCE and self.grasp_cid is None:
  
                ls = p.getLinkState(self.robot, ARMS[arm]["ee"])
                inv_pos, inv_orn = p.invertTransform(ls[4], ls[5])
                rel_pos, rel_orn = p.multiplyTransforms(inv_pos, inv_orn, cube_pos, cube_orn)
                self.grasp_cid = p.createConstraint(self.robot, ARMS[arm]["ee"], self.cube, -1,
                                                    p.JOINT_FIXED, [0, 0, 0], rel_pos, [0, 0, 0],
                                                    parentFrameOrientation=rel_orn)
                p.changeConstraint(self.grasp_cid, maxForce=200)
                self.grasp_arm = arm
                print(f"Objeto agarrado (distancia {dist:.3f} m).")
            else:
                print(f"Pinza cerrada, pero el cubo está a {dist:.3f} m "
                      f"(acércate a menos de {GRASP_DISTANCE} m y vuelve a intentar).")
        else:
            self.grip_closed[arm] = False
            print("Pinza abierta.")
        self._set_fingers_all()

 
    def step(self, state, dt=1.0 / SIM_HZ):
        """Un ciclo de control. dt = tiempo real transcurrido (s), para que la
        velocidad de la pinza sea la misma aunque la ventana vaya lenta."""
        dt = min(dt, 0.05)
        self.last_input = state
        if state["mode"] == "B":
            if state["btn2_pressed"]:
                self.arm = "derecho" if self.arm == "izquierdo" else "izquierdo"
                print(f"Brazo activo: {self.arm}")
            if state["btn1_pressed"]:
                self.toggle_grip()
            vel = np.array([state["x"], state["y"], state["z"]]) * MOVE_SPEED
            t = self.targets[self.arm] + vel * dt
            self.targets[self.arm] = np.clip(t, WS_MIN, WS_MAX)

        self._solve_ik(self.arm)
        self._apply_motors()
   
        for _ in range(max(1, min(4, int(round(dt * SIM_HZ))))):
            p.stepSimulation()
        self._frames = getattr(self, "_frames", 0) + 1
        if self._frames % 6 == 0:
            self._update_text()

    def _update_text(self, force=False):
        tgt = self.targets[self.arm]
        self.txt_target = p.addUserDebugText("TARGET", (tgt + [0, 0, 0.05]).tolist(), [1, 0, 0], 1.5,
                                             replaceItemUniqueId=self.txt_target)
        st = getattr(self, "last_input", None)
        joy = (f"   |   ESP32  X={st['x']:+.2f}  Y={st['y']:+.2f}  Z={st['z']:+.2f}  "
               f"B1={st['btn1']} B2={st['btn2']}") if st else ""
        txt = f"Brazo: {self.arm}   |   objeto: {'AGARRADO' if self.grasp_cid is not None else 'libre'}{joy}"
        if force or getattr(self, "_last_status", None) != txt:
            self.txt_status = p.addUserDebugText(txt, [0.2, 0, 0.95], [0, 0, 0], 1.3,
                                                 replaceItemUniqueId=self.txt_status)
            self._last_status = txt

    def cube_pos(self):
        return np.array(p.getBasePositionAndOrientation(self.cube)[0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None, help="Puerto serial del ESP32, p.ej. COM5")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    console = SerialConsole(port=args.port, baud=args.baud, default_mode="B")
    sim = BaxterTeleop(gui=True)
    print("Mueve la pinza sobre el cubo verde, bájala, BTN1 para agarrar, llévalo al cuadro rojo y BTN1 para soltar.")
    print("BTN2 cambia de brazo.")

    try:
        last = time.time()
        last_print = 0.0
        while p.isConnected():
            now = time.time()
            dt, last = now - last, now
            state = console.read(pybullet_module=p, client_id=sim.client)
            sim.step(state, dt)
            if console.using_esp32 and now - last_print > 1.0:  # diagnóstico en consola
                print(f"ESP32 -> X={state['x']:+.2f} Y={state['y']:+.2f} Z={state['z']:+.2f} "
                      f"BTN1={state['btn1']} BTN2={state['btn2']}   pinza en {np.round(sim.ee_pos(), 2)}")
                last_print = now
            sleep = 1.0 / SIM_HZ - (time.time() - now)
            if sleep > 0:
                time.sleep(sleep)
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        console.close()
        if p.isConnected():
            p.disconnect()
##


if __name__ == "__main__":
    main()
