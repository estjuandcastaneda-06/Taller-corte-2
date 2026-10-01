import os
import sys
import time
import argparse

import numpy as np
import pybullet as p

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(HERE, ".."))
from common.serial_bridge import SerialConsole 

ROBOTS_DATA = os.path.join(HERE, "..", "pybullet_robots", "data")
SIM_HZ = 240
DT = 1.0 / SIM_HZ
CAM_EVERY = 8          
CAM_W, CAM_H = 320, 200

START_POS = [-2.0, 3.0, -0.47]  
START_YAW = 0.0


MAX_SPEED = 0.6        # m/s hacia adelante
MAX_BACK_SPEED = 0.3   # m/s hacia atrás
MAX_TURN = 0.9         # rad/s
ACCEL = 1.2            # m/s^2 (arranque y frenado suaves)
STEP_FREQ = 1.6        # pasos (ciclos de marcha) por segundo a velocidad máxima
HIP_AMP = 0.45         # rad, amplitud del balanceo de cadera a velocidad máxima
KNEE_LIFT = 0.75       # rad, flexión extra de rodilla en la fase de vuelo
ARM_SWING = 0.5        # rad, balanceo de brazos
MAX_STEP_UP = 0.20     # m, escalón más alto que puede subir
OBSTACLE_DIST = 0.45   # m, distancia a la que se detiene frente a un obstáculo
JOINT_SPEED = 1.2      # rad/s en los modos de articulaciones
JOINT_SMOOTH = 3.0     # rad/s, velocidad máx. con la que las articulaciones siguen su objetivo


WALK_BASE = {"l_leg_hpy": -0.25, "r_leg_hpy": -0.25, "l_leg_kny": 0.5, "r_leg_kny": 0.5,
             "l_arm_shx": -1.3, "r_arm_shx": 1.3, "l_arm_elx": 0.4, "r_arm_elx": -0.4}

GROUPS = [
    ("Caminar", None),
    ("Brazo izquierdo", ["l_arm_shz", "l_arm_shx", "l_arm_elx"]),
    ("Brazo derecho", ["r_arm_shz", "r_arm_shx", "r_arm_elx"]),
    ("Pierna izquierda", ["l_leg_hpx", "l_leg_hpy", "l_leg_kny"]),
    ("Pierna derecha", ["r_leg_hpx", "r_leg_hpy", "r_leg_kny"]),
    ("Torso y cabeza", ["back_bkz", "back_bky", "neck_ry"]),
]

POSES = [
    ("Pose T", {}),
    ("Brazos abajo", {"l_arm_shx": -1.3, "r_arm_shx": 1.3}),
    ("Saludo", {"l_arm_shx": -1.3, "r_arm_shx": -1.2, "r_arm_elx": -1.4, "neck_ry": 0.2}),
]


def load_scene():
    """Mismo escenario que atlas.py: laboratorio botlab + cajas Boston Dynamics."""
    p.setAdditionalSearchPath(ROBOTS_DATA)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    atlas = p.loadURDF("atlas/atlas_v4_with_multisense.urdf", START_POS, useFixedBase=True)
    objs = p.loadSDF("botlab/botlab.sdf", globalScaling=2.0)
    zero = [0, 0, 0]
    y2x = p.getQuaternionFromEuler([np.pi / 2., 0, np.pi / 2])
    for o in objs:  
        pos, orn = p.getBasePositionAndOrientation(o)
        newpos, neworn = p.multiplyTransforms(zero, y2x, pos, orn)
        p.resetBasePositionAndOrientation(o, newpos, neworn)
    p.loadURDF("boston_box.urdf", [-2, 3, -2], useFixedBase=True)
    p.loadURDF("boston_box.urdf", [0, 3, -2], useFixedBase=True)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    return atlas


class AtlasConsole:
    def __init__(self, gui=True):
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        self.gui = gui
        if not os.path.isdir(ROBOTS_DATA):
            raise SystemExit(f"No se encontró {ROBOTS_DATA}\nClona "
                             "https://github.com/erwincoumans/pybullet_robots dentro de la carpeta del taller.")
        p.setGravity(0, 0, -10)
        p.setTimeStep(DT)
        
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)

        self.atlas = load_scene()
        for link in range(-1, p.getNumJoints(self.atlas)):
            p.setCollisionFilterGroupMask(self.atlas, link, 0, 0)

        self.jid, self.limits = {}, {}
        for i in range(p.getNumJoints(self.atlas)):
            info = p.getJointInfo(self.atlas, i)
            if info[2] == p.JOINT_REVOLUTE:
                name = info[1].decode()
                self.jid[name] = i
                self.limits[name] = (info[8], info[9])
        self.feet = [i for i in range(p.getNumJoints(self.atlas))
                     if p.getJointInfo(self.atlas, i)[12].decode() in ("l_foot", "r_foot")]
        self.head = self.jid["neck_ry"]

        self.q = {n: 0.0 for n in self.jid}          
        self.manual = {n: 0.0 for n in self.jid}     
        self.group = 0                               
        self.pose = 0
        self.steps = 0
        self.reset_position()

        if gui:
            p.resetDebugVisualizerCamera(3.0, -40, -20, self.pos.tolist())
        self.txt = p.addUserDebugText("", [0, 0, 0], [0, 0, 0], 1.2)
        self._last_txt = None

    def reset_position(self):
        self.pos = np.array(START_POS, dtype=float)
        self.yaw = START_YAW
        self.speed = 0.0
        self.turn = 0.0
        self.vz = 0.0
        self.phase = 0.0
        self.gait_amp = 0.0

    def _ray_down(self, x, y, z_from):
        hit = p.rayTest([x, y, z_from], [x, y, z_from - 5.0])[0]
        return hit[3][2] if hit[0] >= 0 else -2.0

    def _knee_z(self):
        """Altura desde la que se buscan el suelo (así no confunde una mesa con el piso)."""
        return self.pos[2] - 0.95 + MAX_STEP_UP + 0.05

    def _blocked(self, direction):
        """¿Hay una pared/mesa/escalón alto en la dirección de avance?"""
        fwd = np.array([np.cos(self.yaw), np.sin(self.yaw), 0.0]) * np.sign(direction)
        side = np.array([-fwd[1], fwd[0], 0.0])
        foot_z = self.pos[2] - 0.95
        starts, ends = [], []
        for h in (MAX_STEP_UP + 0.05, 0.45, 0.7, 0.95, 1.2, 1.45, 1.7):  
            for lat in (-0.3, 0.0, 0.3):                                   
                start = self.pos + side * lat
                start[2] = foot_z + h
                starts.append(start.tolist())
                ends.append((start + fwd * OBSTACLE_DIST).tolist())
        if any(hit[0] >= 0 for hit in p.rayTestBatch(starts, ends)):
            return True
        ahead = self.pos + fwd * 0.35
        ground_here = self._ray_down(self.pos[0], self.pos[1], self._knee_z())
        ground_ahead = self._ray_down(ahead[0], ahead[1], self._knee_z())
        return ground_ahead - ground_here > MAX_STEP_UP

    def _gait(self, state):
        target_speed = state["y"] * (MAX_SPEED if state["y"] > 0 else MAX_BACK_SPEED)
        target_turn = -state["x"] * MAX_TURN
        if target_speed != 0 and self._blocked(target_speed):
            target_speed = 0.0
    
        self.speed += np.clip(target_speed - self.speed, -ACCEL * DT, ACCEL * DT)
        self.turn += np.clip(target_turn - self.turn, -2 * MAX_TURN * DT, 2 * MAX_TURN * DT)
        if abs(self.speed) < 1e-3 and target_speed == 0:
            self.speed = 0.0

        self.yaw += self.turn * DT
        self.pos[0] += self.speed * np.cos(self.yaw) * DT
        self.pos[1] += self.speed * np.sin(self.yaw) * DT

        activity = min(1.0, max(abs(self.speed) / MAX_SPEED, abs(self.turn) / MAX_TURN * 0.6))
        self.gait_amp += np.clip(activity - self.gait_amp, -2 * DT, 2 * DT)
        direction = -1.0 if self.speed < 0 else 1.0
        self.phase += 2 * np.pi * STEP_FREQ * (0.5 + 0.5 * self.gait_amp) * DT * direction \
            if self.gait_amp > 0.01 else 0.0

        a = self.gait_amp
        target = dict(WALK_BASE)
        for side, ph in (("l", self.phase), ("r", self.phase + np.pi)):
            hip = WALK_BASE[f"{side}_leg_hpy"] - HIP_AMP * a * np.sin(ph) 
            knee = WALK_BASE[f"{side}_leg_kny"] + KNEE_LIFT * a * max(0.0, np.cos(ph) * direction)
            target[f"{side}_leg_hpy"] = hip
            target[f"{side}_leg_kny"] = knee
            target[f"{side}_leg_aky"] = -(hip + knee)                        
        swing = ARM_SWING * a * np.sin(self.phase)
        target["l_arm_shz"] = swing        
        target["r_arm_shz"] = swing
        target["back_bkz"] = -0.08 * a * np.sin(self.phase)
        return target

    def _settle_height(self):
        """Ajusta la altura de la pelvis para que el pie más bajo pise el suelo."""
        feet_bottom = min(p.getAABB(self.atlas, f)[0][2] for f in self.feet)
        ground = max(self._ray_down(*p.getLinkState(self.atlas, f)[4][:2], self._knee_z())
                     for f in self.feet)
        error = ground - feet_bottom            
        if error < -0.02:                     
            self.vz -= 9.81 * DT
            self.pos[2] += max(self.vz * DT, error)
        else:
            self.vz = 0.0
            self.pos[2] += np.clip(error, -0.02, 0.02)

   
    def step(self, state):
        if state["mode"] == "B":
            if state["btn2_pressed"]:
                self.group = (self.group + 1) % len(GROUPS)
                print(f"Modo: {GROUPS[self.group][0]}")
                if self.group != 0:
                    self.manual = {n: self.q[n] for n in self.jid}
            if state["btn1_pressed"]:
                if self.group == 0:
                    self.reset_position()
                    print("Atlas vuelve al punto de partida.")
                else:
                    self.pose = (self.pose + 1) % len(POSES)
                    name, values = POSES[self.pose]
                    self.manual = {n: values.get(n, 0.0) for n in self.jid}
                    print(f"Postura: {name}")

        if self.group == 0:
            target = self._gait(state)
            target = {n: target.get(n, 0.0) for n in self.jid}
        else:
            for axis, joint in zip(("x", "y", "z"), GROUPS[self.group][1]):
                if state[axis] != 0:
                    lo, hi = self.limits[joint]
                    self.manual[joint] = float(np.clip(self.manual[joint] + state[axis] * JOINT_SPEED * DT,
                                                       lo, hi))
            target = self.manual


        max_dq = JOINT_SMOOTH * DT if self.group != 0 else 6.0 * DT
        for n, i in self.jid.items():
            lo, hi = self.limits[n]
            goal = float(np.clip(target[n], lo, hi))
            self.q[n] += float(np.clip(goal - self.q[n], -max_dq, max_dq))
            p.resetJointState(self.atlas, i, self.q[n])

        orn = p.getQuaternionFromEuler([0, 0, self.yaw])
        p.resetBasePositionAndOrientation(self.atlas, self.pos.tolist(), orn)
        self._settle_height()
        p.resetBasePositionAndOrientation(self.atlas, self.pos.tolist(), orn)
        p.stepSimulation()

        if self.gui:
            if self.steps % 4 == 0:
                self._follow_camera()
                self._update_text()
            if self.steps % CAM_EVERY == 0:
                self.render_head_camera()
        self.steps += 1

    def _follow_camera(self):
        cam = p.getDebugVisualizerCamera()   
        p.resetDebugVisualizerCamera(cam[10], cam[8], cam[9], (self.pos + [0, 0, -0.3]).tolist())

    def _update_text(self):
        name, joints = GROUPS[self.group]
        if joints is None:
            txt = f"Modo: CAMINAR   velocidad {self.speed:+.2f} m/s"
        else:
            txt = f"Modo: {name}  (X={joints[0]}  Y={joints[1]}  Z={joints[2]})"
        if txt != self._last_txt:
            self.txt = p.addUserDebugText(txt, [0, 0, 1.25], [0, 0, 0], 1.2,
                                          parentObjectUniqueId=self.atlas, replaceItemUniqueId=self.txt)
            self._last_txt = txt

    def render_head_camera(self):
        """Cámara sintética en la cabeza (MultiSense) -> paneles RGB, Depth y Segmentation."""
        ls = p.getLinkState(self.atlas, self.head)
        pos, orn = np.array(ls[4]), ls[5]
        rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        fwd, up = rot[:, 0], rot[:, 2]
        eye = pos + 0.25 * fwd + 0.05 * up
        view = p.computeViewMatrix(eye.tolist(), (eye + fwd).tolist(), up.tolist())
        proj = p.computeProjectionMatrixFOV(fov=70, aspect=CAM_W / CAM_H, nearVal=0.05, farVal=30)
        return p.getCameraImage(CAM_W, CAM_H, view, proj, renderer=p.ER_BULLET_HARDWARE_OPENGL)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None, help="Puerto serial del ESP32, p.ej. COM7")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    console = SerialConsole(port=args.port, baud=args.baud, default_mode="B")
    sim = AtlasConsole(gui=True)
    print("MODO CAMINAR: joystick adelante/atrás = caminar, izquierda/derecha = girar.")
    print("BTN2 = cambiar de modo (articulaciones), BTN1 = volver al inicio / siguiente postura.")

    try:
        while p.isConnected():
            t0 = time.time()
            state = console.read(pybullet_module=p, client_id=sim.client)
            sim.step(state)
            dt = DT - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        console.close()
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
