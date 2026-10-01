"""
drone_control.py
-----------------
Parte A del taller: mover un ENJAMBRE de drones simulados de un punto A a un
punto B y a un punto C, con el control gestionado desde el ESP32.

Se apoya en el repositorio  https://github.com/utiasDSL/gym-pybullet-drones
(entorno CtrlAviary + controlador DSLPIDControl, igual que examples/pid.py).

Uso (con el entorno conda "taller" activado):
    python drone_control.py                      # sin ESP32: modo teclado
    python drone_control.py --port COM5          # con ESP32 en ese puerto
    python drone_control.py --num_drones 4       # menos drones (PC lento)
    python drone_control.py --auto False         # solo avanza con BTN1

Controles (ESP32 en modo "D"  /  teclado):
    Joystick X/Y  (flechas)        -> desplaza la formación en X/Y
    Eje Z         (RePag/AvPag, U/J)-> sube / baja la formación
    BTN1          (ESPACIO)        -> ir al siguiente punto (A -> B -> C -> A ...)
    BTN2          (ENTER o B)      -> activar / desactivar modo automático

Lógica:
    - Los drones despegan del suelo y forman un círculo alrededor del
      "centro de formación".
    - El centro de formación viaja SUAVEMENTE (velocidad limitada) hacia el
      punto activo; cada dron sigue su posición relativa con un PID.
    - En modo automático, al llegar a un punto esperan HOLD_TIME segundos y
      siguen al siguiente; al llegar a C se quedan en hover.
    - Se guarda un log en drones/flight_log.csv como evidencia.
"""

import os
import sys
import time
import argparse
import numpy as np

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common.serial_bridge import SerialConsole  # noqa: E402

try:
    import pybullet as p
    from gym_pybullet_drones.utils.enums import DroneModel, Physics
    from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
    from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
    from gym_pybullet_drones.utils.utils import sync, str2bool
except ImportError as e:
    raise SystemExit(
        "No se pudo importar gym-pybullet-drones o una de sus dependencias.\n"
        "Activa el entorno correcto:  conda activate taller\n"
        "y si falta algo:  pip install transforms3d scipy matplotlib pyserial\n"
        f"Detalle: {e}"
    )

# --- Puntos A, B y C de la misión (centro de la formación, en metros) -------
WAYPOINTS = {
    "A": np.array([0.0, 0.0, 1.0]),
    "B": np.array([1.5, 1.5, 1.3]),
    "C": np.array([-1.5, 1.5, 0.8]),
}
WP_NAMES = list(WAYPOINTS.keys())
WP_COLORS = {"A": [0, 0.8, 0], "B": [0, 0, 1], "C": [1, 0, 0]}

FORMATION_RADIUS = 0.45   # m, radio del círculo de drones
CRUISE_SPEED = 0.6        # m/s, velocidad máxima del centro de formación
JOYSTICK_SPEED = 0.8      # m/s, velocidad con la que el joystick mueve la formación
ARRIVE_THRESHOLD = 0.12   # m, error medio para considerar que llegó
HOLD_TIME = 2.0           # s que esperan en cada punto (modo automático)


def formation_offsets(n, radius):
    if n == 1:
        return np.zeros((1, 3))
    return np.array([[radius * np.cos(2 * np.pi * i / n),
                      radius * np.sin(2 * np.pi * i / n),
                      0.0] for i in range(n)])


def draw_waypoints(client):
    """Dibuja los puntos A, B, C y la ruta entre ellos en la GUI."""
    pts = [WAYPOINTS[k] for k in WP_NAMES]
    for k in WP_NAMES:
        wp = WAYPOINTS[k]
        # poste vertical desde el suelo + etiqueta
        p.addUserDebugLine([wp[0], wp[1], 0], wp.tolist(), WP_COLORS[k], 2, physicsClientId=client)
        p.addUserDebugText(k, (wp + [0, 0, 0.15]).tolist(), WP_COLORS[k], textSize=2.0,
                           physicsClientId=client)
    for a, b in zip(pts, pts[1:]):
        p.addUserDebugLine(a.tolist(), b.tolist(), [0.6, 0.6, 0.6], 1, physicsClientId=client)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None, help="Puerto serial del ESP32, p.ej. COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--num_drones", type=int, default=8)
    parser.add_argument("--auto", type=str2bool, default=True,
                        help="Recorrer A->B->C automáticamente (BTN2 lo alterna)")
    parser.add_argument("--obstacles", type=str2bool, default=True)
    parser.add_argument("--gui", type=str2bool, default=True)
    parser.add_argument("--duration", type=float, default=0,
                        help="Segundos de simulación (0 = hasta cerrar la ventana)")
    args = parser.parse_args()

    console = SerialConsole(port=args.port, baud=args.baud, default_mode="D")

    n = args.num_drones
    offsets = formation_offsets(n, FORMATION_RADIUS)
    init_xyzs = offsets + np.array([0.0, 0.0, 0.1])  # en el suelo, debajo del punto A

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=n,
        initial_xyzs=init_xyzs,
        initial_rpys=np.zeros((n, 3)),
        physics=Physics.PYB,
        pyb_freq=240,
        ctrl_freq=48,
        gui=args.gui,
        obstacles=args.obstacles,
        user_debug_gui=False,
    )
    client = env.CLIENT
    ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(n)]
    dt = env.CTRL_TIMESTEP

    obs, _ = env.reset()
    if args.gui:
        p.resetDebugVisualizerCamera(cameraDistance=4.0, cameraYaw=-30, cameraPitch=-30,
                                     cameraTargetPosition=[0, 0.8, 0.6], physicsClientId=client)
        draw_waypoints(client)
        status_id = p.addUserDebugText("", [0, 0, 2.2], [0, 0, 0], textSize=1.4,
                                       physicsClientId=client)

    wp_idx = 0
    goal = WAYPOINTS[WP_NAMES[wp_idx]].copy()   # punto hacia donde va la formación
    center = np.array([0.0, 0.0, 0.1])          # centro de formación comandado (suave)
    auto = args.auto
    arrived_since = None

    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "flight_log.csv")
    log_file = open(log_path, "w")
    log_file.write("t,waypoint,cx,cy,cz," + ",".join(f"x{i},y{i},z{i}" for i in range(n)) + "\n")

    print(f"{n} drones. Ruta: A -> B -> C  |  modo automático: {auto}")
    print("BTN1/ESPACIO = siguiente punto, BTN2/ENTER = auto on/off, joystick/flechas = mover formación.")

    action = np.zeros((n, 4))
    start = time.time()
    step = 0
    last_status = ""
    try:
        while True:
            if args.gui and not p.isConnected(client):
                break

            # ---------------- Entrada de la consola (ESP32 o teclado) ----------
            state = console.read(pybullet_module=p, client_id=client)
            if state["mode"] == "D":
                if state["btn2_pressed"]:
                    auto = not auto
                    arrived_since = None
                    print(f"Modo automático: {auto}")
                if state["btn1_pressed"]:
                    wp_idx = (wp_idx + 1) % len(WP_NAMES)
                    goal = WAYPOINTS[WP_NAMES[wp_idx]].copy()
                    arrived_since = None
                    print(f"-> Yendo al punto {WP_NAMES[wp_idx]}: {goal}")
                joy = np.array([state["x"], state["y"], state["z"]])
                if np.any(joy != 0):
                    goal += joy * JOYSTICK_SPEED * dt
                    goal[2] = np.clip(goal[2], 0.3, 3.0)

            # ---------------- Trayectoria suave del centro de formación --------
            delta = goal - center
            dist = np.linalg.norm(delta)
            max_step = CRUISE_SPEED * dt
            center = goal.copy() if dist <= max_step else center + delta / dist * max_step

            # ---------------- Simulación + PID de cada dron ---------------------
            obs, _, terminated, truncated, _ = env.step(action)
            positions = obs[:, 0:3]
            targets = center + offsets
            for j in range(n):
                action[j, :], _, _ = ctrls[j].computeControlFromState(
                    control_timestep=dt, state=obs[j], target_pos=targets[j])

            # ---------------- ¿Llegó la formación al punto activo? -------------
            t = step * dt  # tiempo simulado
            err = np.mean(np.linalg.norm(positions - (goal + offsets), axis=1))
            if err < ARRIVE_THRESHOLD:
                if arrived_since is None:
                    arrived_since = t
                    print(f"Formación en el punto {WP_NAMES[wp_idx]} (error medio {err:.3f} m)")
                elif auto and wp_idx < len(WP_NAMES) - 1 and t - arrived_since > HOLD_TIME:
                    wp_idx += 1
                    goal = WAYPOINTS[WP_NAMES[wp_idx]].copy()
                    arrived_since = None
                    print(f"-> Yendo al punto {WP_NAMES[wp_idx]}: {goal}")

            # ---------------- Log + texto en pantalla ---------------------------
            log_file.write(f"{t:.3f},{WP_NAMES[wp_idx]},{center[0]:.3f},{center[1]:.3f},{center[2]:.3f},"
                           + ",".join(f"{q[0]:.3f},{q[1]:.3f},{q[2]:.3f}" for q in positions) + "\n")
            if args.gui:
                txt = (f"Destino: {WP_NAMES[wp_idx]}  |  auto: {'ON' if auto else 'OFF'}  |  "
                       f"{'ESP32' if console.using_esp32 else 'teclado'}")
                if txt != last_status:
                    status_id = p.addUserDebugText(txt, [0, 0, 2.2], [0, 0, 0], textSize=1.4,
                                                   replaceItemUniqueId=status_id,
                                                   physicsClientId=client)
                    last_status = txt
                sync(step, start, dt)  # tiempo real

            step += 1
            if args.duration and t >= args.duration:
                break
            if terminated:
                break

    except KeyboardInterrupt:
        pass
    finally:
        log_file.close()
        console.close()
        try:
            env.close()
        except Exception:
            pass
        print(f"Log de vuelo guardado en: {log_path}")


if __name__ == "__main__":
    main()
