from ctypes import Structure
import tkinter as tk
import math
import random
import csv
from datetime import datetime

class AeroSim:
    def __init__(self, root):
        self.root = root
        self.root.title("AERO_CORE")
        self.root.geometry("1400x850")
        self.root.configure(bg="#000000")

        self.rho = 1.225
        self.cd = 1.0
        self.frontal_area = 0.0
        self.drag_force = 0.0
        self.downforce = 0.0

        self.session_data = []

        self.drawing = False
        self.last_x, self.last_y = 0, 0
        self.obstacles = []
        self.simulating = False

        self.num_particles = 1200
        self.p_pos = []
        self.p_vel = []
        self.p_ids = []

        self.build_ui()
        self.init_particles()

    def build_ui(self):

        left_panel = tk.Frame(self.root, bg="#000000", width=400, highlightbackground="#FFFFFF", highlightthickness=2)
        left_panel.pack(side="left", fill="y", padx=10, pady=10)
        left_panel.pack_propagate(False)

        tk.Label(left_panel, text="[ AERO_CORE_V11 ]", font=("Courier", 24, "bold"), fg="#FFFFFF", bg="#000000").pack(pady=(20, 5))
        tk.Label(left_panel, text="DRAG.TO.DRAW", font=("Courier", 15), fg="#888888", bg="#000000").pack(pady=(0, 20))

        ctrl_frame = tk.Frame(left_panel, bg="#000000", padx=15, pady=15, highlightbackground="#FFFFFF", highlightthickness=1)
        ctrl_frame.pack(fill="x", padx=15, pady=10)

        self.sim_btn = tk.Button(ctrl_frame, text="INITIATE TUNNEL", command=self.toggle_sim, bg="#FFFFFF", fg="#000000", font=("Courier", 14, "bold"), relief="flat", pady=8)
        self.sim_btn.pack(fill="x", pady=(0, 10))

        tk.Button(ctrl_frame, text="[ EXPORT CSV TELEMETRY ]", command=self.export_data, bg="#000000", fg="#00FFCC", font=("Courier", 12, "bold"), highlightbackground="#00FFCC", highlightthickness=1, relief="flat", pady=5).pack(fill="x", pady=(0, 10))

        tk.Button(ctrl_frame, text="CLEAR WORKSPACE", command=self.clear_canvas, bg="#000000", fg="#FFFFFF", font=("Courier", 10, "bold"), highlightbackground="#FFFFFF", highlightthickness=1, relief="flat", pady=5).pack(fill="x")

        tk.Label(ctrl_frame, text="WIND VELOCITY [m/s]", fg="#FFFFFF", bg="#000000", font=("Courier", 10, "bold")).pack(anchor="w", pady=(15, 5))
        self.vel_var = tk.DoubleVar(value=50.0)
        tk.Scale(ctrl_frame, variable=self.vel_var, from_=10, to=120, orient="horizontal", bg="#000000", fg="#FFFFFF", highlightthickness=0, bd=0, troughcolor="#333333", font=("Courier", 10, "bold")).pack(fill="x")

        math_frame = tk.Frame(left_panel, bg="#000000", padx=15, pady=15, highlightbackground="#FFFFFF", highlightthickness=1)
        math_frame.pack(fill="x", padx=15, pady=10)

        tk.Label(math_frame, text="// LIVE FLUID DYNAMICS", fg="#FFFFFF", bg="#000000", font=("Courier", 10, "bold")).pack(anchor="w")

        img_canvas = tk.Canvas(math_frame, bg="#000000", width=250, height=60, highlightthickness=0)
        img_canvas.pack(anchor="w", pady=5)

        txt_color = "#FFFFFF"

        img_canvas.create_text(20, 30, text="F", font=("Georgia", 24, "italic"), fill=txt_color)
        img_canvas.create_text(35, 40, text="D", font=("Georgia", 14, "italic"), fill=txt_color)
        img_canvas.create_text(60, 30, text="=", font=("Georgia", 24), fill=txt_color)
        img_canvas.create_text(90, 30, text="½", font=("Georgia", 24), fill=txt_color)
        img_canvas.create_text(115, 30, text="ρ", font=("Georgia", 24, "italic"), fill=txt_color)
        img_canvas.create_text(135, 30, text="v", font=("Georgia", 24, "italic"), fill=txt_color)
        img_canvas.create_text(147, 18, text="2", font=("Georgia", 12), fill=txt_color)
        img_canvas.create_text(170, 30, text="C", font=("Georgia", 24, "italic"), fill=txt_color)
        img_canvas.create_text(185, 40, text="D", font=("Georgia", 14, "italic"), fill=txt_color)
        img_canvas.create_text(210, 30, text="A", font=("Georgia", 24, "italic"), fill=txt_color)

        self.lbl_vel = self.create_data_row(math_frame, "BASE VELOCITY :", "50.0 m/s")
        self.lbl_v_top = self.create_data_row(math_frame, "UPPER AIR V.  :", "0.0 m/s")
        self.lbl_v_bot = self.create_data_row(math_frame, "LOWER AIR V.  :", "0.0 m/s")

        tk.Frame(math_frame, bg="#FFFFFF", height=2).pack(fill="x", pady=15)

        tk.Label(math_frame, text="AERODYNAMIC DRAG (F_D)", font=("Courier", 9, "bold"), fg="#888888", bg="#000000").pack(anchor="w")
        self.lbl_force = tk.Label(math_frame, text="0 N", font=("Courier", 24, "bold"), fg="#FF3333", bg="#000000")
        self.lbl_force.pack(anchor="w")

        tk.Label(math_frame, text="BERNOULLI DOWNFORCE (F_L)", font=("Courier", 9, "bold"), fg="#888888", bg="#000000").pack(anchor="w", pady=(10,0))
        self.lbl_downforce = tk.Label(math_frame, text="0 N", font=("Courier", 24, "bold"), fg="#00FFCC", bg="#000000")
        self.lbl_downforce.pack(anchor="w")


        self.canvas_frame = tk.Frame(self.root, bg="#000000", highlightbackground="#FFFFFF", highlightthickness=2)
        self.canvas_frame.pack(side="right", fill="both", expand=True, padx=(0, 10), pady=10)

        self.canvas = tk.Canvas(self.canvas_frame, bg="#000000", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        self.root.bind("<Configure>", self.draw_grid)
        self.canvas.bind("<Button-1>", self.start_draw)
        self.canvas.bind("<B1-Motion>", self.drag_draw)
        self.canvas.bind("<ButtonRelease-1>", self.end_draw)

    def create_data_row(self, parent, label_text, val_text):
        row = tk.Frame(parent, bg="#000000")
        row.pack(fill="x", pady=2)
        tk.Label(row, text=label_text, font=("Courier", 10, "bold"), fg="#888888", bg="#000000").pack(side="left")
        val_label = tk.Label(row, text=val_text, font=("Courier", 10, "bold"), fg="#FFFFFF", bg="#000000")
        val_label.pack(side="right")
        return val_label

    def draw_grid(self, event=None):
        self.canvas.delete("grid")
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        for i in range(0, w, 20):
            color = "#111111" if i % 100 != 0 else "#333333"
            self.canvas.create_line(i, 0, i, h, fill=color, tags="grid")
        for i in range(0, h, 20):
            color = "#111111" if i % 100 != 0 else "#333333"
            self.canvas.create_line(0, i, w, i, fill=color, tags="grid")
        self.canvas.tag_lower("grid")

    def init_particles(self):
        for _ in range(self.num_particles):
            self.p_pos.append([-100, -100])
            self.p_vel.append([0, 0])
            line_id = self.canvas.create_line(0, 0, 0, 0, fill="#FFFFFF", width=2, tags="particle")
            self.p_ids.append(line_id)

    def clear_canvas(self):
        self.canvas.delete("obstacle")
        self.obstacles.clear()
        self.session_data.clear()
        self.update_physics_data()

    def export_data(self):
        if not self.session_data:
            return
        filename = f"AeroTelemetry_{datetime.now().strftime('%H%M%S')}.csv"
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time_Step", "Base_Velocity_ms", "Drag_N", "Downforce_N", "Upper_Air_V", "Lower_Air_V", "Frontal_Area"])
            writer.writerows(self.session_data)

        self.lbl_force.config(text="EXPORTED!")
        self.root.after(1500, lambda: self.lbl_force.config(text=f"{self.drag_force:,.0f} N"))

    def toggle_sim(self):
        self.simulating = not self.simulating
        if self.simulating:
            self.sim_btn.config(text="HALT TUNNEL", bg="#000000", fg="#FFFFFF", highlightbackground="#FFFFFF", highlightthickness=2)
            h = self.canvas.winfo_height()
            for i in range(self.num_particles):
                self.p_pos[i] = [random.uniform(-600, 0), random.uniform(20, h-20)]
                self.p_vel[i] = [random.uniform(20, 30), 0]
            self.update_loop()
        else:
            self.sim_btn.config(text="INITIATE TUNNEL", bg="#FFFFFF", fg="#000000", highlightthickness=0)

    def start_draw(self, event):
        if self.simulating: return
        self.drawing = True
        self.last_x, self.last_y = event.x, event.y

    def drag_draw(self, event):
        if self.simulating or not self.drawing: return
        dx, dy = event.x - self.last_x, event.y - self.last_y
        length = math.hypot(dx, dy)

        if length > 5:
            self.canvas.create_line(self.last_x, self.last_y, event.x, event.y,
                                    fill="#FFFFFF", width=8, tags="obstacle", capstyle=tk.ROUND)

            tx, ty = dx / length, dy / length
            nx, ny = -ty, tx
            if nx > 0: nx, ny = -nx, -ny

            self.obstacles.append({
                'x1': self.last_x, 'y1': self.last_y, 'x2': event.x, 'y2': event.y,
                'length': length, 'nx': nx, 'ny': ny, 'tx': tx, 'ty': ty
            })
            self.last_x, self.last_y = event.x, event.y

    def end_draw(self, event):
        if self.simulating or not self.drawing: return
        self.drawing = False
        self.update_physics_data()

    def update_physics_data(self):
        if not self.obstacles:
            self.frontal_area = 0.0
            return

        min_y, max_y = float('inf'), float('-inf')
        for obs in self.obstacles:
            min_y = min(min_y, obs['y1'], obs['y2'])
            max_y = max(max_y, obs['y1'], obs['y2'])

        self.mid_y = (min_y + max_y) / 2
        pixel_height = max_y - min_y
        self.frontal_area = (pixel_height * 0.005) * 1.5

    def get_heat_color(self, speed, base_speed):
        """Calculates particle color: Red=Fast, Blue=Slow"""
        ratio = speed / (base_speed * 1.5)
        ratio = max(0, min(1, ratio))

        r = int(ratio * 255)
        g = int((1 - ratio) * 200)
        b = int((1 - ratio) * 255)
        return f"#{r:02x}{g:02x}{b:02x}"


    def update_loop(self):
        if not self.simulating: return

        v = self.vel_var.get()
        self.lbl_vel.config(text=f"{v:.1f} m/s")

        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        base_wind_speed = v * 0.8
        influence_radius = 60.0

        v_top_sum, v_bot_sum = 0, 0
        top_count, bot_count = 0, 0

        for i in range(self.num_particles):
            px, py = self.p_pos[i]
            vx, vy = self.p_vel[i]

            vx = vx * 0.75 + base_wind_speed * 0.25
            vy = vy * 0.90

            for obs in self.obstacles:
                v_px, v_py = px - obs['x1'], py - obs['y1']
                t_proj = v_px * obs['tx'] + v_py * obs['ty']

                if 0 <= t_proj <= obs['length']:
                    dist = abs(v_px * obs['nx'] + v_py * obs['ny'])
                  
                    if dist < influence_radius:
                        
                        force = (influence_radius - dist) / influence_radius
                        
                        vx = vx * (1 - force) + (obs['tx'] * base_wind_speed * force * 1.2)
                       
                        vy = vy * (1 - force) + (obs['ty'] * base_wind_speed * force * 1.2)
                       
                        vx += obs['nx'] * force * 15
                        
                        vy += obs['ny'] * force * 15


            px += vx
            py += vy

            if self.obstacles and (self.mid_y - 150 < py < self.mid_y + 150):
                mag = math.hypot(vx, vy)
                if py < self.mid_y:
                    v_top_sum += mag
                    top_count += 1
                else:
                    v_bot_sum += mag
                    bot_count += 1

            if px > w or py < -50 or py > h + 50:
                px, py = random.uniform(-100, -10), random.uniform(20, h-20)
                vx, vy = base_wind_speed, 0

            self.p_pos[i] = [px, py]
            self.p_vel[i] = [vx, vy]

            color = self.get_heat_color(math.hypot(vx, vy), base_wind_speed)
            self.canvas.itemconfig(self.p_ids[i], fill=color)
            self.canvas.coords(self.p_ids[i], px - vx*1.8, py - vy*1.8, px, py)

        avg_v_top = (v_top_sum / top_count) if top_count > 0 else base_wind_speed
        avg_v_bot = (v_bot_sum / bot_count) if bot_count > 0 else base_wind_speed

        self.lbl_v_top.config(text=f"{avg_v_top:.1f} m/s")
        self.lbl_v_bot.config(text=f"{avg_v_bot:.1f} m/s")

        self.drag_force = 0.5 * self.rho * (v**2) * self.cd * self.frontal_area
        self.lbl_force.config(text=f"{self.drag_force:,.0f} N")

        self.downforce = 0.5 * self.rho * ((avg_v_bot**2) - (avg_v_top**2)) * (max(0.1, self.frontal_area * 2))
        self.lbl_downforce.config(text=f"{self.downforce:,.0f} N")

        if len(self.session_data) < 5000:
            self.session_data.append([len(self.session_data), v, self.drag_force, self.downforce, avg_v_top, avg_v_bot, self.frontal_area])

        self.root.after(8, self.update_loop)

if __name__ == "__main__":
    root = tk.Tk()
    app = AeroSim(root)
    root.mainloop()
