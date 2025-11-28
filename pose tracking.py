#%%
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

#%% util
def wrapToPi(angle):
    """Normalisasi sudut ke rentang (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

def transformMat(alpha, beta, gamma, tx, ty, tz):
    # 1. Rotasi dasar
    rotx = np.array([
        [1, 0, 0],
        [0, math.cos(alpha), -math.sin(alpha)],
        [0, math.sin(alpha),  math.cos(alpha)]
    ])
    roty = np.array([
        [ math.cos(beta), 0, math.sin(beta)],
        [0, 1, 0],
        [-math.sin(beta), 0, math.cos(beta)]
    ])
    rotz = np.array([
        [math.cos(gamma), -math.sin(gamma), 0],
        [math.sin(gamma),  math.cos(gamma), 0],
        [0, 0, 1]
    ])
    # 2. Rotasi total R = Rx * Ry * Rz
    rot_total = rotx @ roty
    rot_total = rot_total @ rotz

    # 3. Vektor translasi
    trans_vector = np.array([[tx],
                             [ty],
                             [tz]])
    # 4. Susun 3x4
    R_t_3x4  = np.hstack((rot_total, trans_vector))
    # 5. Baris homogen
    homogeneous_row = np.array([[0, 0, 0, 1]])
    # 6. Matriks transformasi 4x4
    transform_matrix_4x4 = np.vstack((R_t_3x4, homogeneous_row))
    return transform_matrix_4x4

#%% koneksi ke CoppeliaSim
print("Program Started")

client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(False)
sim.startSimulation()

wR_Handle   = sim.getObject("/rightMotor")
wL_Handle   = sim.getObject("/leftMotor")
s3_Handle   = sim.getObject("/ultrasonicSensor[3]")  # belum dipakai
p3dx_Handle = sim.getObject("/PioneerP3DX")
disc_Handle = sim.getObject("/Disc")

# buffer data
d_xyyaw     = []
d_t         = []
dat_disc2rob = np.zeros((4, 1))
dat_errors   = np.zeros((3, 1))

t_prv = 0.0

sim.addLog(1, "get vel start")
time.sleep(2)
start_time = time.time()

# ====== PARAMETER ROBOT & KONTROL ======
rw = 0.195 / 2  # radius roda (m)
rb = 0.381 / 2  # half wheelbase / body radius (m)

# jarak switching posisi → orientasi (m), tuning sendiri
d_sw = 0.05

kp_lin_base = 0.5   # gain linear dasar
kp_ang_max  = 0.8   # gain untuk heading (positional orientation)
kp_ori_max  = 20.0   # gain untuk orientasi akhir (orientation tracking)

# ====== MAIN LOOP ======
while True:
    t_now = time.time() - start_time
    if t_now > 10:
        break

    # --- Pose robot & disk di world frame ---
    bod_pos_xyz = sim.getObjectPosition(p3dx_Handle, sim.handle_world)
    bod_pos_abg = sim.getObjectOrientation(p3dx_Handle, sim.handle_world)

    disc_pos_xyz = sim.getObjectPosition(disc_Handle, sim.handle_world)
    disc_pos_abg = sim.getObjectOrientation(disc_Handle, sim.handle_world)

    # Jadikan homogen
    disc_pos_xyz_hom = np.array([
        [disc_pos_xyz[0]],
        [disc_pos_xyz[1]],
        [disc_pos_xyz[2]],
        [1]
    ])

    # --- Transform world → body ---
    # T_w_b  (body relatif world)
    disc2body_mat = transformMat(
        0.0,                   # roll diabaikan (2D)
        0.0,                   # pitch diabaikan (2D)
        bod_pos_abg[2],        # yaw robot
        bod_pos_xyz[0],        # x robot
        bod_pos_xyz[1],        # y robot
        bod_pos_xyz[2]         # z robot (boleh 0 juga kalau semua di lantai)
    )

    # p_d^b = T_b_w * p_d^w  (T_b_w = inv(T_w_b))
    disc2body_pos = np.linalg.inv(disc2body_mat) @ disc_pos_xyz_hom

    x_db = float(disc2body_pos[0, 0])
    y_db = float(disc2body_pos[1, 0])

    # ====== ERROR CALCULATION ======
    # 1) error posisi di sumbu x robot (target di depan robot)
    ed = np.array([x_db])   # 1x1 array

    # 2) error heading (arah ke target di frame robot)
    eh_angle = math.atan2(y_db, x_db)
    eh = np.array([eh_angle])

    # 3) error orientasi akhir (yaw disk - yaw robot, dibungkus ke (-pi, pi])
    yaw_robot = bod_pos_abg[2]
    yaw_disc  = disc_pos_abg[2]
    eo_angle = wrapToPi(yaw_disc - yaw_robot)
    eo = np.array([eo_angle])

    # 4) jarak Euclidean robot–disk (di world)
    dx = disc_pos_xyz[0] - bod_pos_xyz[0]
    dy = disc_pos_xyz[1] - bod_pos_xyz[1]
    abs_d = math.sqrt(dx*dx + dy*dy)

    errors = np.vstack((ed, eh, eo))   # 3x1

    # ====== MODE SWITCHING POSISI / ORIENTASI ======
    # mode = exp(-abs_d / d_sw)
    # abs_d >> d_sw → mode ~ 0  (fokus ke heading)
    # abs_d ~ 0      → mode ~ 1  (fokus ke orientasi akhir)
    mode = math.exp(-abs_d / d_sw)

    # gain linear bisa dibuat konstan atau sedikit dikurangi saat dekat target
    kp_lin = kp_lin_base

    # gain heading (positional orientation): besar saat jauh (mode kecil)
    kp_ang = kp_ang_max * (1.0 - mode)

    # gain orientasi akhir: besar saat dekat (mode besar)
    kp_ori = kp_ori_max * mode

    # ====== KONTROL v, ω ======
    # v = kecepatan linear
    # w = kombinasi koreksi heading dan orientasi
    v = ed * kp_lin
    w = eh * kp_ang + eo * kp_ori

    vel_vec = np.vstack((v, w))  # [v; w] 2x1

    # ====== KINEMATIKA DIFFERENTIAL DRIVE ======
    # [vR]   [1  rb][v]
    # [vL] = [1 -rb][w]
    kin_mat = np.array([
        [1.0,  rb],
        [1.0, -rb]
    ])

    v_rl = kin_mat @ vel_vec   # v_rl[0]=vR, v_rl[1]=vL (m/s)

    vR = float(v_rl[0, 0])
    vL = float(v_rl[1, 0])

    # (opsional) pembatasan kecepatan
    v_max = 0.6   # m/s, silakan tuning
    wR = vR / rw
    wL = vL / rw

    # ====== ACTUATION ======
    sim.setJointTargetVelocity(wR_Handle, wR)  # rad/s
    sim.setJointTargetVelocity(wL_Handle, wL)

    # ====== SAVE DATA ======
    dat_disc2rob = np.hstack((dat_disc2rob, disc2body_pos))
    dat_errors   = np.hstack((dat_errors, errors))

    d_xyyaw.append([
        bod_pos_xyz[0],
        bod_pos_xyz[1],
        bod_pos_abg[2]
    ])
    d_t.append(t_now)

    t_prv = t_now

    # logging ke console CoppeliaSim
    sim.addLog(1,
        f"t={t_now:5.2f}s | ed={ed[0]:.3f} m | "
        f"eh={math.degrees(eh[0]):.2f} deg | "
        f"eo={math.degrees(eo[0]):.2f} deg | "
        f"d={abs_d:.3f} m | mode={mode:.2f}"
    )

# ====== STOP ======
sim.setJointTargetVelocity(wR_Handle, 0.0)
sim.setJointTargetVelocity(wL_Handle, 0.0)
sim.addLog(1, "sim com ended")

# convert to np array
dat_xyyaw   = np.array(d_xyyaw)
dat_t       = np.array(d_t)
dat_disc2rob = dat_disc2rob[:, 1:]  # remove first zero column
dat_errors   = dat_errors[:, 1:]

# normalisasi yaw ke (-pi, pi]
dat_xyyaw[:, 2] = np.arctan2(np.sin(dat_xyyaw[:, 2]),
                             np.cos(dat_xyyaw[:, 2]))

# %% PLOT XY
plt.figure(figsize=(8, 6))
plt.plot(dat_xyyaw[:, 0], dat_xyyaw[:, 1],
         linewidth=2, label='$^wB$ traj')
plt.scatter(dat_xyyaw[0, 0], dat_xyyaw[0, 1],
            marker='o', s=100, color='red', label='Start')
plt.scatter(dat_xyyaw[-1, 0], dat_xyyaw[-1, 1],
            marker='x', s=100, color='green', label='End')
plt.xlabel('$x_w$ (m)', fontsize=12)
plt.ylabel('$y_w$ (m)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()
now = datetime.now()
filename = now.strftime("%y%m%d%H%M_xy_track") + ".svg"
# plt.savefig(filename, format='svg')
print(f"Plot XY saved as '{filename}'")

# %% PLOT e_d(t)
plt.figure(figsize=(8, 6))
plt.plot(dat_t, dat_errors[0, :],
         linewidth=2, label='$e_d$')
plt.xlabel('$t$ (sec)', fontsize=12)
plt.ylabel('$e_d$ (m)', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.6)
plt.legend()
now = datetime.now()
filename = now.strftime("%y%m%d%H%M_ed_track") + ".svg"
# plt.savefig(filename, format='svg')
print(f"Plot e_d saved as '{filename}'")

# %% PLOT e_h(t)
plt.figure(figsize=(8, 6))
plt.plot(dat_t, np.rad2deg(dat_errors[1, :]),
         linewidth=2, label='$e_h$')
plt.xlabel('$t$ (sec)', fontsize=12)
plt.ylabel('$e_h$ (deg)', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.6)
plt.legend()
now = datetime.now()
filename = now.strftime("%y%m%d%H%M_eh_track") + ".svg"
# plt.savefig(filename, format='svg')
print(f"Plot e_h saved as '{filename}'")

plt.show()
