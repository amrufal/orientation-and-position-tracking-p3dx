# Deskripsi

Program ini mengimplementasikan **orientation and position tracking (pose alignment)** untuk robot mobile **Pioneer P3DX** di **CoppeliaSim** menggunakan Python dan **ZMQ Remote API**.

Tujuan program:

- Membuat robot **mendekati objek target** (`/Disc`) hingga jarak ke objek kecil.
- Membuat robot **menyelaraskan orientasi (yaw)** robot dengan orientasi objek.
- Menunjukkan cara praktis:
  - menggunakan **transformasi koordinat** (world → body) dengan matriks homogen 4×4,
  - mendefinisikan **galat posisi dan orientasi**,
  - dan mengontrol robot **differential-drive** berdasarkan galat tersebut.

Program akan:
1. Membaca pose robot dan objek dari CoppeliaSim.
2. Menghitung posisi objek di **frame robot** (body frame).
3. Menghitung error posisi dan orientasi (`e_d`, `e_h`, `e_o`).
4. Menghasilkan kecepatan linear dan sudut `(v, ω)` yang dikonversi menjadi kecepatan roda kanan–kiri.
5. Mengirim perintah kecepatan ke motor roda P3DX.

---

# Fitur

- **Koneksi ke CoppeliaSim via ZMQ Remote API**
  - Memulai dan mengontrol simulasi langsung dari script Python.
  - Mengambil handle:
    - base robot: `/PioneerP3DX`
    - roda kanan: `/rightMotor`
    - roda kiri: `/leftMotor`
    - objek target: `/Disc`

- **Transformasi koordinat (world → body)**
  - Membentuk matriks transformasi homogen dari pose robot (`x, y, z, yaw`).
  - Menghitung matriks invers untuk mendapatkan transformasi world → body.
  - Mengubah posisi objek dari world frame ke body frame robot (koordinat relatif terhadap robot).

- **Definisi galat pose**
  - `e_d` — galat jarak sepanjang sumbu depan robot (komponen x pada frame robot).
  - `e_h` — galat heading, yaitu sudut arah objek terhadap sumbu depan robot:
    - dihitung menggunakan `atan2(y_db, x_db)`.
  - `e_o` — galat orientasi akhir (yaw objek dikurangi yaw robot), dinormalisasi dengan fungsi `wrapToPi` agar berada di rentang (-π, π].

- **Mode switching eksponensial posisi–orientasi**
  - Menggunakan fungsi:
    - `mode = exp(-d / d_sw)`  
      di mana `d` adalah jarak Euclidean robot–objek dan `d_sw` adalah jarak switching.
  - Fungsi ini mengatur fokus kontrol:
    - saat jauh dari objek → fokus mengurangi `e_h` (menghadapkan robot ke objek),
    - saat dekat objek → fokus mengurangi `e_o` (menyelaraskan orientasi akhir dengan objek).
  - Gain di-interpolasi secara halus:
    - `kp_ang` (heading) besar saat jauh, kecil saat dekat,
    - `kp_ori` (orientasi akhir) kecil saat jauh, besar saat dekat.

- **Kontrol differential-drive**
  - Hukum kontrol:
    - `v = kp_lin * e_d`
    - `ω = kp_ang * e_h + kp_ori * e_o`
  - Konversi ke kecepatan linear roda (m/s):
    - menggunakan matriks:
      - `[v_R]   = [1  r_b] [v]`
      - `[v_L]     [1 -r_b] [ω]`
  - Konversi ke kecepatan sudut roda (rad/s):
    - `ω_R = v_R / r_w`
    - `ω_L = v_L / r_w`
  - Mengirim kecepatan roda ke CoppeliaSim dengan `setJointTargetVelocity`.

- **Pencatatan dan visualisasi**
  - Menyimpan:
    - lintasan robot `(x_w(t), y_w(t), yaw(t))`,
    - posisi objek dalam frame robot,
    - galat `e_d(t)`, `e_h(t)`, dan (opsional) `e_o(t)`,
    - waktu `t`.
  - Menampilkan grafik:
    - lintasan robot di bidang `x_w–y_w`,
    - kurva `e_d(t)` (galat jarak),
    - kurva `e_h(t)` (galat heading dalam derajat).

```bash
pip install numpy matplotlib coppeliasim-zmqremoteapi-client
