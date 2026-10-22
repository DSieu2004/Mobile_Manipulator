"""Persistent Tk canvas that queues user strokes for the MuJoCo robot."""

import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import time
import tkinter as tk

from interactive_reference import generate_strokes


WIDTH = 680
HEIGHT = 380
ROOT = Path(__file__).resolve().parents[1]


class DrawingPanel:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("Draw for the mobile manipulator")
        self.root.resizable(False, False)
        self.root.configure(bg="#e8edf2")
        tk.Label(self.root, text="Vẽ các nét trên bảng, rồi bấm Gửi cho robot",
                 font=("Sans", 14, "bold"), bg="#e8edf2").pack(pady=(14, 5))
        tk.Label(self.root, text="Trái → phải: y = +0,34 → −0,55 m    |    Trên → dưới: z = 1,00 → 0,58 m",
                 bg="#e8edf2").pack(pady=(0, 8))
        self.canvas = tk.Canvas(self.root, width=WIDTH, height=HEIGHT, bg="white",
                                highlightthickness=1, highlightbackground="#9aa8b4")
        self.canvas.pack(padx=20)
        self.canvas.bind("<ButtonPress-1>", self.start_stroke)
        self.canvas.bind("<B1-Motion>", self.move_stroke)
        self.canvas.bind("<ButtonRelease-1>", self.end_stroke)
        buttons = tk.Frame(self.root, bg="#e8edf2")
        buttons.pack(pady=10)
        tk.Button(buttons, text="Hoàn tác nét", command=self.undo, width=13).pack(side=tk.LEFT, padx=4)
        tk.Button(buttons, text="Xóa bảng", command=self.clear, width=13).pack(side=tk.LEFT, padx=4)
        tk.Button(buttons, text="Gửi cho robot", command=self.submit, width=17,
                  bg="#167a45", fg="white").pack(side=tk.LEFT, padx=4)
        tk.Button(buttons, text="Kết thúc / Thoát MuJoCo", command=self.close, width=22,
                  bg="#a9343d", fg="white").pack(side=tk.LEFT, padx=4)
        self.status = tk.StringVar(value="Vẽ nhiều nét bằng chuột. Có thể gửi nhiều lần; robot xử lý theo thứ tự.")
        tk.Label(self.root, textvariable=self.status, bg="#e8edf2", wraplength=WIDTH).pack(pady=(0, 12))
        self.strokes: list[list[list[float]]] = []
        self.current: list[list[float]] | None = None
        self.submissions = 0
        self.messages: queue.Queue[str] = queue.Queue()
        self.pending: queue.Queue[tuple[int, list[list[list[float]]]] | None] = queue.Queue()
        self.active: subprocess.Popen | None = None
        self.closed = False
        threading.Thread(target=self.start_backend, daemon=True).start()
        self.root.after(100, self.show_status)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    @staticmethod
    def point(event: tk.Event) -> list[float]:
        return [min(1.0, max(0.0, event.x / WIDTH)),
                min(1.0, max(0.0, event.y / HEIGHT))]

    def start_stroke(self, event: tk.Event) -> None:
        self.current = [self.point(event)]
        self.canvas.create_oval(event.x - 2, event.y - 2, event.x + 2, event.y + 2,
                                fill="#c71f35", outline="")

    def move_stroke(self, event: tk.Event) -> None:
        if self.current is None:
            return
        point = self.point(event)
        old = self.current[-1]
        if (point[0] - old[0]) ** 2 * WIDTH ** 2 + (point[1] - old[1]) ** 2 * HEIGHT ** 2 < 4:
            return
        self.current.append(point)
        self.canvas.create_line(old[0] * WIDTH, old[1] * HEIGHT,
                                point[0] * WIDTH, point[1] * HEIGHT,
                                fill="#c71f35", width=3, capstyle=tk.ROUND, joinstyle=tk.ROUND)

    def end_stroke(self, event: tk.Event) -> None:
        if self.current is not None:
            self.strokes.append(self.current)
            self.current = None

    def redraw(self) -> None:
        self.canvas.delete("all")
        for stroke in self.strokes:
            for left, right in zip(stroke, stroke[1:]):
                self.canvas.create_line(left[0] * WIDTH, left[1] * HEIGHT,
                                        right[0] * WIDTH, right[1] * HEIGHT,
                                        fill="#c71f35", width=3, capstyle=tk.ROUND, joinstyle=tk.ROUND)
            if len(stroke) == 1:
                x, y = stroke[0]
                self.canvas.create_oval(x * WIDTH - 2, y * HEIGHT - 2,
                                        x * WIDTH + 2, y * HEIGHT + 2,
                                        fill="#c71f35", outline="")

    def undo(self) -> None:
        if self.strokes:
            self.strokes.pop()
            self.redraw()

    def clear(self) -> None:
        self.strokes.clear()
        self.canvas.delete("all")

    def submit(self) -> None:
        if self.current is not None:
            self.strokes.append(self.current)
            self.current = None
        if not self.strokes:
            self.status.set("Hãy vẽ ít nhất một nét trước khi gửi.")
            return
        try:
            generate_strokes(self.strokes)
        except ValueError as error:
            self.status.set(str(error))
            return
        self.submissions += 1
        self.pending.put((self.submissions, [stroke.copy() for stroke in self.strokes]))
        self.clear()
        self.status.set(f"Đã xếp hàng bản vẽ #{self.submissions}. Bạn có thể vẽ và gửi tiếp.")

    def read_backend_output(self) -> None:
        assert self.active is not None and self.active.stdout is not None
        recent = []
        for line in self.active.stdout:
            recent.append(line.rstrip())
            recent = recent[-8:]
            if line.startswith("PANEL_STATUS "):
                try:
                    self.messages.put(json.loads(line[len("PANEL_STATUS "):])["status"])
                except (ValueError, KeyError):
                    pass
        if not self.closed:
            self.messages.put("MuJoCo đã dừng ngoài ý muốn. " + " | ".join(recent[-3:]))

    def start_backend(self) -> None:
        output_dir = ROOT / "runs"
        output_dir.mkdir(exist_ok=True)
        command = [str(ROOT / "scripts/run_ocs2_vr.sh"), "--interactive"]
        if os.environ.get("DRAW_VIEWER_FPS"):
            command.extend(("--viewer-fps", os.environ["DRAW_VIEWER_FPS"]))
        try:
            self.active = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.PIPE,
                                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                           text=True, bufsize=1, start_new_session=True)
            threading.Thread(target=self.read_backend_output, daemon=True).start()
            while not self.closed and self.active.poll() is None:
                job = self.pending.get()
                if job is None or self.closed:
                    return
                number, strokes = job
                stamp = time.strftime("%Y%m%d_%H%M%S")
                drawing = output_dir / f"drawing_{stamp}_{number:03d}.json"
                log = drawing.with_suffix(".csv")
                drawing.write_text(json.dumps({"strokes": strokes}), encoding="utf-8")
                assert self.active.stdin is not None
                self.active.stdin.write(json.dumps({"type": "draw", "id": number,
                                                    "strokes": strokes, "log": str(log)}) + "\n")
                self.active.stdin.flush()
                self.messages.put(f"Đã gửi bản #{number} tới MuJoCo. Có thể gửi thêm bản khác.")
                self.pending.task_done()
        except (OSError, BrokenPipeError) as error:
            if not self.closed:
                self.messages.put(f"Không thể kết nối MuJoCo: {error}")

    def show_status(self) -> None:
        while not self.messages.empty():
            try:
                self.status.set(self.messages.get_nowait())
            except queue.Empty:
                pass
        self.root.after(100, self.show_status)

    def close(self) -> None:
        self.closed = True
        if self.active is not None and self.active.poll() is None:
            try:
                assert self.active.stdin is not None
                self.active.stdin.write('{"type":"quit"}\n')
                self.active.stdin.flush()
            except (OSError, BrokenPipeError):
                pass
        self.pending.put(None)
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


if __name__ == "__main__":
    DrawingPanel().run()
