"""Interactive webcam, DA3 relative depth and cloud in one viewer."""

import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium")

with app.setup:
    import marimo as mo
    from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.session import Workbench
    from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.depth_pipeline import DepthPipeline
    from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.viser_view import ViserViewer
    from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.polyscope_view import PolyscopeViewer


@app.cell
def model():
    depth_pipeline = DepthPipeline(
        resolution=336, calibration="build/webcam_session/calibration_refined.json")
    return (depth_pipeline,)


@app.cell
def camera_session():
    import os as _os
    import time as _time
    import cv2 as _cv2

    _path = _os.environ.get('PERCEPTION_REPLAY_IMAGE')
    if _path:
        _image = _cv2.imread(_path)
        if _image is None or _image.shape[:2] != (360, 640):
            raise ValueError('Replay requires a readable 640x360 raw image for this calibration')

        def _source():
            _time.sleep(1 / 30)
            return _image.copy()

        workbench = Workbench(source=_source)
    else:
        workbench = Workbench(camera=0)
    workbench.configure(stride=2)
    return (workbench,)


@app.cell
def face_model():
    import cv2
    import numpy as np
    import subprocess
    import struct
    import json
    import atexit
    import os
    from pathlib import Path
    from textwrap import dedent

    # Run MediaPipe in the prepared environment with its compatible protobuf.
    # This worker receives images only. The notebook remains the sole camera owner.
    face_worker_source = dedent(r'''
    import sys, struct, json, time
    import numpy as np
    import mediapipe as mp
    model = mp.tasks.vision.FaceLandmarker.create_from_options(
        mp.tasks.vision.FaceLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path="build/webcam_session/face_landmarker.task"),
            running_mode=mp.tasks.vision.RunningMode.VIDEO, num_faces=3,
            min_face_detection_confidence=0.5, min_face_presence_confidence=0.5))
    def send(value):
        data = json.dumps(value).encode()
        sys.stdout.buffer.write(struct.pack("!I", len(data)) + data)
        sys.stdout.buffer.flush()
    def read_exact(size):
        data = bytearray()
        while len(data) < size:
            part = sys.stdin.buffer.read(size-len(data))
            if not part:
                raise EOFError()
            data.extend(part)
        return data
    send({"connections": sorted(mp.solutions.face_mesh.FACEMESH_CONTOURS)})
    try:
        while True:
            h, w = struct.unpack("!II", read_exact(8))
            bgr = np.frombuffer(read_exact(h*w*3), dtype=np.uint8).reshape(h,w,3)
            rgb = np.ascontiguousarray(bgr[:,:,::-1])
            result = model.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), time.monotonic_ns()//1000000)
            send([[[p.x,p.y,p.z] for p in face] for face in result.face_landmarks])
    except EOFError:
        model.close()
    ''')
    Path("build/webcam_session/rendering_evidence").mkdir(parents=True, exist_ok=True)
    face_worker_log = open("build/webcam_session/rendering_evidence/face_worker.log", "ab", buffering=0)
    face_worker = subprocess.Popen([
        str(Path("roscon_uk_2026/next_steps/webcam_session/toolbox/people/.pixi/envs/default/bin/python").resolve()),
        "-u", "-c", face_worker_source], env={**os.environ, "PYTHONPATH": ""}, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=face_worker_log)
    atexit.register(face_worker.terminate)

    def read_face_response():
        def read_exact(size):
            data = bytearray()
            while len(data) < size:
                part = face_worker.stdout.read(size-len(data))
                if not part:
                    raise RuntimeError("Face worker stopped. See face_worker.log")
                data.extend(part)
            return data
        size, = struct.unpack("!I", read_exact(4))
        return json.loads(read_exact(size))

    face_connections = np.array(read_face_response()["connections"], dtype=int)

    def detect_faces(image):
        h, w = image.shape[:2]
        face_worker.stdin.write(struct.pack("!II", h, w))
        face_worker.stdin.write(np.ascontiguousarray(image).tobytes())
        face_worker.stdin.flush()
        return read_face_response()

    # Canonical centimetres: convert x right, y up, z outward to camera axes.
    canonical_face = np.array([
        [float(v) for v in line.split()[1:4]]
        for line in Path("build/webcam_session/canonical_face_model.obj").read_text().splitlines()
        if line.startswith("v ")], dtype=np.float64) * [1, -1, -1]
    face_pose_indices = np.array([1, 4, 6, 10, 33, 133, 168, 263, 362, 61, 291, 152, 234, 454])

    return (
        canonical_face,
        cv2,
        detect_faces,
        face_connections,
        face_pose_indices,
        np,
    )


@app.cell
def hand_model(np):
    from roscon_uk_2026.next_steps.webcam_session.toolbox.people.adapters import People
    import time as hand_clock

    # Prepared OpenCV Zoo palm detector and 21-landmark hand-pose model.
    hand_model = People("hand")
    hand_connections = np.array([
        [0,1],[1,2],[2,3],[3,4],
        [0,5],[5,6],[6,7],[7,8],
        [5,9],[9,10],[10,11],[11,12],
        [9,13],[13,14],[14,15],[15,16],
        [13,17],[0,17],[17,18],[18,19],[19,20]], dtype=int)

    return People, hand_clock, hand_connections, hand_model


@app.cell
def hand_drag_logic(np):
    class FistDrag:
        """Logical press/hold/release for Viser panning. No desktop mouse events."""

        def __init__(self):
            self.held = False
            self.xy = None
            self.handedness = None
            self.pending = 0
            self.pending_xy = None
            self.presses = 0
            self.releases = 0
            self.moves = 0
            self.event = "released"
            self.reason = "Ready: close a hand to grab the view"
            self.curled = 0

        @staticmethod
        def measure(hand, image_size):
            landmarks = np.asarray(hand["landmarks"], dtype=float)
            if landmarks.shape != (21,3) or not np.isfinite(landmarks).all():
                return None
            width,height = image_size
            palm = landmarks[[5,9,13,17],:2]
            if np.any(palm<0) or np.any(palm>np.array([width-1,height-1])):
                return None
            if np.linalg.norm(landmarks[5,:2]-landmarks[17,:2]) < 15:
                return None
            curled = 0
            for mcp,pip,dip,tip in [(5,6,7,8),(9,10,11,12),(13,14,15,16),(17,18,19,20)]:
                a,b = landmarks[mcp]-landmarks[pip], landmarks[dip]-landmarks[pip]
                cosine = np.dot(a,b)/max(np.linalg.norm(a)*np.linalg.norm(b),1e-6)
                angle = np.degrees(np.arccos(np.clip(cosine,-1,1)))
                folded = np.linalg.norm(landmarks[tip]-landmarks[mcp]) < 1.3*np.linalg.norm(a)
                curled += int(angle<130 or folded)
            return dict(xy=palm.mean(axis=0)/[width,height], curled=curled,
                        handedness=float(hand.get("right_hand_score",0.5)),
                        confidence=float(hand.get("confidence",0.0)))

        def release(self, reason):
            if self.held:
                self.releases += 1
            self.held = False
            self.xy = None
            self.handedness = None
            self.pending = 0
            self.pending_xy = None
            self.event = "released"
            self.reason = reason

        def update(self, hands, image_size, clients, gain=1.0, enabled=True):
            if not enabled:
                self.release("Hand drag disabled")
                return
            measurements = [m for hand in hands if (m:=self.measure(hand,image_size)) is not None]
            if self.held:
                # Lock by position and handedness. Never hand a held drag to the
                # other hand when the controlling hand disappears.
                matches = [m for m in measurements
                           if abs(m["handedness"]-self.handedness)<0.65
                           and np.linalg.norm(m["xy"]-self.xy)<0.18]
                if not matches:
                    self.release("Hand lost or out of view")
                    return
                hand = min(matches,key=lambda m:np.linalg.norm(m["xy"]-self.xy))
                self.curled = hand["curled"]
                if hand["curled"]<2:
                    self.release("Hand opened")
                    return
                # Suppress small tracking jitter and excessive jumps.
                delta = hand["xy"]-self.xy
                if np.linalg.norm(delta)<0.003:
                    self.event = "held"
                    return
                self.xy = hand["xy"].copy()
                delta = delta*min(1.,0.07/max(np.linalg.norm(delta),1e-6))
                for client in clients.values():
                    camera = client.camera
                    forward = camera.look_at-camera.position
                    distance = np.linalg.norm(forward)
                    if distance<1e-6:
                        continue
                    forward /= distance
                    right = np.cross(forward,camera.up_direction)
                    right /= max(np.linalg.norm(right),1e-6)
                    up = np.cross(right,forward)
                    view_height = 2*distance*np.tan(camera.fov/2)
                    shift = gain*view_height*(-delta[0]*camera.aspect*right+delta[1]*up)
                    # Viser's position setter translates look_at too, preserving
                    # the current orientation and orbit distance.
                    with client.atomic():
                        camera.position = camera.position+shift
                self.moves += 1
                self.event = "dragging"
                self.reason = "Closed hand: move to pan; open to release"
                return
            closed = [m for m in measurements if m["curled"]>=3]
            if not closed:
                self.pending = 0
                self.pending_xy = None
                self.reason = "Ready: close a hand to grab the view"
                self.curled = max((m["curled"] for m in measurements),default=0)
                return
            hand = max(closed,key=lambda m:m["confidence"])
            if self.pending_xy is not None and np.linalg.norm(hand["xy"]-self.pending_xy)<0.12:
                self.pending += 1
            else:
                self.pending = 1
            self.pending_xy = hand["xy"].copy()
            self.curled = hand["curled"]
            if self.pending>=2:
                self.held = True
                self.xy = hand["xy"].copy()
                self.handedness = hand["handedness"]
                self.presses += 1
                self.event = "pressed"
                self.reason = "Closed hand: move to pan; open to release"

        def status(self):
            return dict(held=self.held,event=self.event,reason=self.reason,
                        curled_fingers=int(self.curled),presses=self.presses,
                        releases=self.releases,moves=self.moves,
                        xy=None if self.xy is None else self.xy.tolist())


    class TwoHandDrag(FistDrag):
        """One fist pans. Two fists change camera distance around a fixed target."""

        def __init__(self):
            super().__init__()
            self.mode = "released"
            self.zoom_pair = None
            self.zoom_pending = None
            self.zoom_pending_frames = 0
            self.zoom_separation = None
            self.zoom_limits = {}
            self.zoom_moves = 0

        def release(self, reason):
            super().release(reason)
            self.mode = "released"
            self.zoom_pair = None
            self.zoom_pending = None
            self.zoom_pending_frames = 0
            self.zoom_separation = None
            self.zoom_limits = {}

        @staticmethod
        def separation(pair, image_size):
            width,height = image_size
            return float(np.linalg.norm((pair[0]["xy"]-pair[1]["xy"])*[width/height,1.]))

        @staticmethod
        def match_pair(previous, measurements):
            matches = []
            for a in measurements:
                for b in measurements:
                    if a is b:
                        continue
                    pair = [a,b]
                    distances = [np.linalg.norm(p["xy"]-q["xy"]) for p,q in zip(previous,pair)]
                    if (max(distances)<0.22 and
                        all(abs(p["handedness"]-q["handedness"])<0.65 for p,q in zip(previous,pair))):
                        matches.append((sum(distances),pair))
            return min(matches,key=lambda item:item[0])[1] if matches else None

        def update(self, hands, image_size, clients, gain=1., enabled=True,
                   zoom_enabled=True, zoom_gain=1.):
            if not enabled:
                self.release("Hand gestures disabled")
                return
            measurements = [m for hand in hands if (m:=self.measure(hand,image_size)) is not None]
            if self.mode=="zoom":
                if not zoom_enabled:
                    self.release("Two-hand zoom disabled")
                    return
                pair = self.match_pair(self.zoom_pair,measurements)
                if pair is None or any(m["curled"]<2 for m in pair):
                    self.release("Zoom released: hand opened or lost")
                    return
                separation = self.separation(pair,image_size)
                if separation<0.07:
                    self.release("Zoom released: hands too close to track")
                    return
                self.zoom_pair = pair
                self.xy = (pair[0]["xy"]+pair[1]["xy"])/2
                self.curled = min(m["curled"] for m in pair)
                log_ratio = float(np.log(self.zoom_separation/separation))
                if abs(log_ratio)<0.012:
                    self.event = "held"
                    return
                factor = float(np.exp(np.clip(log_ratio*zoom_gain,-0.15,0.15)))
                self.zoom_separation = separation
                for client_id,client in clients.items():
                    camera = client.camera
                    target = camera.look_at.copy()
                    offset = camera.position-target
                    distance = float(np.linalg.norm(offset))
                    if distance<1e-6:
                        continue
                    if client_id not in self.zoom_limits:
                        self.zoom_limits[client_id] = (max(0.01,distance*0.15),distance*8)
                    near,far = self.zoom_limits[client_id]
                    next_distance = float(np.clip(distance*factor,near,far))
                    with client.atomic():
                        camera.position = target+offset*(next_distance/distance)
                        camera.look_at = target
                self.zoom_moves += 1
                self.event = "zooming"
                self.reason = "Two fists: apart zooms in; together zooms out"
                return
            closed = sorted([m for m in measurements if m["curled"]>=3],
                            key=lambda m:m["confidence"],reverse=True)
            if zoom_enabled and len(closed)>=2:
                pair = self.match_pair(self.zoom_pending,closed) if self.zoom_pending else None
                self.zoom_pending_frames = self.zoom_pending_frames+1 if pair else 1
                pair = pair or closed[:2]
                self.zoom_pending = pair
                separation = self.separation(pair,image_size)
                if separation>=0.09 and self.zoom_pending_frames>=2:
                    self.release("Switching to two-hand zoom")
                    self.held = True
                    self.mode = "zoom"
                    self.zoom_pair = pair
                    self.zoom_separation = separation
                    self.xy = (pair[0]["xy"]+pair[1]["xy"])/2
                    self.curled = min(m["curled"] for m in pair)
                    self.presses += 1
                    self.event = "pressed"
                    self.reason = "Two fists: apart zooms in; together zooms out"
                # Pause panning while a second closed hand is being confirmed.
                return
            self.zoom_pending = None
            self.zoom_pending_frames = 0
            super().update(hands,image_size,clients,gain=gain,enabled=enabled)
            self.mode = "pan" if self.held else "released"

        def status(self):
            return dict(super().status(),mode=self.mode,zoom_moves=self.zoom_moves,
                        hand_separation=self.zoom_separation)


    class TolerantHandDrag(TwoHandDrag):
        """Hold a pan or zoom through two missed processed frames."""

        def __init__(self):
            super().__init__()
            self.missed_frames = 0
            self.tolerated_misses = 0
            self.recoveries = 0

        def release(self, reason):
            super().release(reason)
            self.missed_frames = 0

        def update(self, hands, image_size, clients, gain=1., enabled=True,
                   zoom_enabled=True, zoom_gain=1.):
            if not enabled or (self.mode=="zoom" and not zoom_enabled):
                self.release("Hand gesture disabled")
                return
            if self.held:
                measurements = [m for hand in hands if (m:=self.measure(hand,image_size)) is not None]
                tracks = self.zoom_pair if self.mode=="zoom" else [
                    dict(xy=self.xy,handedness=self.handedness)]
                available = list(measurements)
                matched = []
                # Allow more displacement after a gap, while retaining hand identity.
                gate = (0.22 if self.mode=="zoom" else 0.18)*(1+0.5*self.missed_frames)
                for track in tracks:
                    candidates = [m for m in available
                                  if abs(m["handedness"]-track["handedness"])<0.65
                                  and np.linalg.norm(m["xy"]-track["xy"])<gate]
                    hand = min(candidates,key=lambda m:np.linalg.norm(m["xy"]-track["xy"])) if candidates else None
                    if hand is not None:
                        # Remove by identity; measurement dictionaries contain arrays.
                        available = [m for m in available if m is not hand]
                        if hand["curled"]<2:
                            self.release("Hand opened")
                            return
                    matched.append(hand)
                if any(hand is None for hand in matched):
                    self.missed_frames += 1
                    if self.missed_frames>=3:
                        self.release("Hand lost for 3 consecutive frames")
                    else:
                        self.tolerated_misses += 1
                        self.event = "tracking_gap"
                        self.reason = f"Holding {self.mode}: missed {self.missed_frames}/2 frames; movement paused"
                    return
                if self.missed_frames:
                    # Rebase on the reacquired position. Do not apply motion that
                    # happened while tracking was missing.
                    if self.mode=="zoom":
                        separation = self.separation(matched,image_size)
                        if separation<0.07:
                            self.release("Hands too close to track")
                            return
                        self.zoom_pair = matched
                        self.zoom_separation = separation
                        self.xy = (matched[0]["xy"]+matched[1]["xy"])/2
                    else:
                        self.xy = matched[0]["xy"].copy()
                    self.curled = min(m["curled"] for m in matched)
                    self.missed_frames = 0
                    self.recoveries += 1
                    self.event = "held"
                    self.reason = "Tracking recovered; grab retained"
                    return
            super().update(hands,image_size,clients,gain=gain,enabled=enabled,
                           zoom_enabled=zoom_enabled,zoom_gain=zoom_gain)

        def status(self):
            return dict(super().status(),missed_frames=self.missed_frames,
                        tolerated_misses=self.tolerated_misses,recoveries=self.recoveries)


    return FistDrag, TwoHandDrag, TolerantHandDrag


@app.cell
def pose_model(People, np):
    # Retain the prepared person detector and 33-joint pose model.
    pose_model = People("pose")
    pose_connections = np.array([
        [0,1],[1,2],[2,3],[3,7],[0,4],[4,5],[5,6],[6,8],[9,10],
        [11,12],[11,13],[13,15],[15,17],[15,19],[15,21],[17,19],
        [12,14],[14,16],[16,18],[16,20],[16,22],[18,20],
        [11,23],[12,24],[23,24],
        [23,25],[25,27],[27,29],[27,31],[29,31],
        [24,26],[26,28],[28,30],[28,32],[30,32]], dtype=int)

    return pose_connections, pose_model


@app.cell
def foreground_surface(
    cv2,
    face_legend,
    face_live_status,
    np,
    viewer,
    workbench,
):
    from roscon_uk_2026.next_steps.webcam_session.geometry import masked_depth_points

    workbench.configure(foreground_only=True, foreground_depth_margin=0.5)
    foreground_status = viewer.server.gui.add_markdown("Foreground: waiting for a face.")
    foreground_status.order = face_live_status.order - 0.01
    foreground_help = viewer.server.gui.add_markdown(
        "**Foreground:** depth surface seeded by the nearest detected face, plus nearby "
        "detected hands. Increase **foreground_depth_margin** to retain more behind the face; "
        "decrease it to remove more background. The margin is a fraction of the depth gap "
        "from the face to the farther scene. Toggle **foreground_only** for the full scene. "
        "Depth is relative. Objects touching the person at similar depth can remain.")
    foreground_help.order = face_legend.order + 0.001

    def face_surface_mask(depth, faces, hands, image_size, state, margin):
        height,width = depth.shape
        image_width,image_height = image_size
        valid = np.isfinite(depth)&(depth>0)
        smooth = cv2.medianBlur(np.where(valid,depth,0).astype(np.float32),5)
        yy,xx = np.ogrid[:height,:width]

        def face_region(box):
            x0,y0,x1,y1 = box
            # Exclude face-box edges, which can contain background or hair.
            cx,cy = (x0+x1)/2,(y0+y1)/2
            rx,ry = max((x1-x0)*0.27,2),max((y1-y0)*0.30,2)
            return ((xx-cx)/rx)**2+((yy-cy)/ry)**2<=1

        choices = []
        for face in faces:
            uv = np.asarray(face,dtype=float)[:,:2]*[width,height]
            if len(uv)<3 or not np.isfinite(uv).all():
                continue
            box = np.r_[uv.min(axis=0),uv.max(axis=0)]
            region = face_region(box)&valid
            samples = smooth[region & (smooth>0)]
            if len(samples)>=8:
                choices.append((float(np.median(samples)),box,region))
        held_face = False
        if choices:
            anchor,box,seed = min(choices,key=lambda item:item[0])
            state["foreground_face_box"] = box.copy()/[width,height,width,height]
            state["foreground_face_misses"] = 0
        else:
            misses = state.get("foreground_face_misses",0)+1
            state["foreground_face_misses"] = misses
            previous = state.get("foreground_face_box")
            if previous is None or misses>2:
                return np.zeros_like(valid),dict(face_found=False,face_held=False,reason="Waiting for a face")
            box = previous*[width,height,width,height]
            seed = face_region(box)&valid
            samples = smooth[seed & (smooth>0)]
            if len(samples)<8:
                return np.zeros_like(valid),dict(face_found=False,face_held=False,reason="No valid face depth")
            anchor = float(np.median(samples))
            held_face = True
        background_depth = float(np.percentile(smooth[valid & (smooth>0)],90))
        depth_gap = max(background_depth-anchor,anchor*0.03)
        cutoff = anchor+float(np.clip(margin,0.01,1.0))*depth_gap
        candidate = valid & (smooth>0) & (smooth<=cutoff) & (depth<=cutoff)
        count,labels = cv2.connectedComponents(candidate.astype(np.uint8),connectivity=8)
        seed_labels,counts = np.unique(labels[seed & candidate],return_counts=True)
        positive = seed_labels>0
        if not np.any(positive):
            return np.zeros_like(valid),dict(face_found=True,face_held=held_face,reason="No connected face surface")
        face_label = int(seed_labels[positive][np.argmax(counts[positive])])
        selected_labels = {face_label}
        # A hand may be separated from the torso in the image. Include only its
        # component inside the same face-derived foreground depth limit.
        for hand in hands:
            palm = np.asarray(hand,dtype=float)[4:67].reshape(21,3)[[0,5,9,13,17],:2]
            palm = palm*[width/image_width,height/image_height]
            for x,y in np.rint(palm).astype(int):
                if 0<=x<width and 0<=y<height and labels[y,x]>0:
                    selected_labels.add(int(labels[y,x]))
        mask = np.isin(labels,list(selected_labels)) & valid
        return mask,dict(face_found=True,face_held=held_face,face_depth=anchor,
                         depth_cutoff=cutoff,background_depth=background_depth,face_box_depth_grid=box.tolist(),
                         components=len(selected_labels),kept_pixels=int(mask.sum()),
                         total_pixels=int(valid.sum()),reason="Face-seeded foreground")

    def extract_foreground(packet, image, faces, hands, state, controls):
        if not controls.get("foreground_only",True):
            foreground_status.content = "**Foreground: off** | Full scene"
            packet["foreground"] = dict(enabled=False)
            return packet,image
        depth = packet["depth"]
        mask,info = face_surface_mask(depth,faces,hands,(image.shape[1],image.shape[0]),
                                     state,controls.get("foreground_depth_margin",0.5))
        # Reuse the prepared native masked depth unprojection. Colors come from
        # the unannotated DA3 image on this exact depth grid and source frame.
        selected = np.zeros_like(mask)
        stride = max(1,int(controls.get("stride",2)))
        selected[::stride,::stride] = mask[::stride,::stride]
        points,pixels = masked_depth_points(depth,selected,packet["intrinsics"],far=1e6)
        rgb = packet["image"][...,::-1]
        packet["points"] = points
        packet["colors"] = rgb.reshape(-1,3)[pixels].copy()
        packet["foreground_mask"] = mask
        packet["foreground"] = dict(info,enabled=True,kept_points=len(points))
        depth_image = packet["depth_image"].copy()
        depth_image[~mask] = 0
        packet["depth_image"] = depth_image
        full_mask = cv2.resize(mask.astype(np.uint8),(image.shape[1],image.shape[0]),
                               interpolation=cv2.INTER_NEAREST).astype(bool)
        image = image.copy()
        image[~full_mask] = (24,24,24)
        if info.get("face_found"):
            suffix = " | Face briefly occluded" if info.get("face_held") else ""
            foreground_status.content = (
                f"**Foreground:** {len(points):,} points | "
                f"{100*mask.mean():.0f}% of image{suffix}")
        else:
            foreground_status.content = f"**Foreground:** {info['reason']}"
        return packet,image


    return extract_foreground, foreground_help, foreground_status


@app.cell
def raised_hand_filter(cv2, foreground_status, np, viewer, workbench):
    workbench.configure(raised_hand_only=True)
    raised_hand_status = viewer.server.gui.add_markdown("Raise either hand above its shoulder to appear.")
    raised_hand_status.order = foreground_status.order - 0.01

    def select_raised_people(faces, hands, poses, image_size, controls):
        width,height = image_size
        threshold = float(controls.get("pose_confidence",0.5))
        selected = []
        heads = []
        for index,pose in enumerate(poses):
            lm = np.asarray(pose[1][:33],dtype=float)
            visible = (np.isfinite(lm).all(axis=1) & (lm[:,3]>=threshold)
                       & (lm[:,4]>=threshold) & (lm[:,0]>=0)&(lm[:,0]<width)
                       & (lm[:,1]>=0)&(lm[:,1]<height))
            span = float(np.linalg.norm(lm[11,:2]-lm[12,:2]))
            margin = max(5.,span*0.05)
            raised = float(pose[-1])>=threshold and any(
                visible[s] and visible[w] and lm[w,1]<lm[s,1]-margin
                for s,w in [(11,15),(12,16)])
            if raised:
                selected.append(index)
            head = lm[0,:2] if visible[0] else lm[[11,12],:2].mean(axis=0)-[0,span*0.5]
            heads.append(head)
        mask = np.zeros((height,width),np.uint8)
        for index in selected:
            person_mask = np.asarray(poses[index][3],dtype=np.uint8)
            if person_mask.shape != mask.shape:
                person_mask = cv2.resize(person_mask,(width,height),interpolation=cv2.INTER_NEAREST)
            mask |= (person_mask>0).astype(np.uint8)
        kept_faces = []
        for face in faces:
            uv = np.asarray(face)[:,:2]*[width,height]
            center = (uv.min(axis=0)+uv.max(axis=0))/2
            if not heads:
                continue
            distances = np.linalg.norm(np.asarray(heads)-center,axis=1)
            nearest = int(np.argmin(distances))
            size = max(float(np.linalg.norm(uv.max(axis=0)-uv.min(axis=0))),20.)
            if nearest in selected and distances[nearest]<size:
                kept_faces.append(face)
                cv2.fillConvexPoly(mask,cv2.convexHull(np.rint(uv).astype(np.int32)),1)
        kept_hands = []
        for hand in hands:
            uv = np.asarray(hand[4:67]).reshape(21,3)[:,:2]
            center = uv[[0,5,9,13,17]].mean(axis=0)
            matches = []
            for index,pose in enumerate(poses):
                lm = np.asarray(pose[1])
                for wrist in [15,16]:
                    if lm[wrist,3]>=threshold and lm[wrist,4]>=threshold:
                        matches.append((float(np.linalg.norm(lm[wrist,:2]-center)),index))
            if matches:
                distance,index = min(matches)
                hand_size = max(float(np.linalg.norm(uv.max(axis=0)-uv.min(axis=0))),30.)
                if index in selected and distance<hand_size:
                    kept_hands.append(hand)
                    cv2.fillConvexPoly(mask,cv2.convexHull(np.rint(uv).astype(np.int32)),1)
        # Include a small boundary around the coarse model segmentation.
        mask = cv2.dilate(mask,np.ones((5,5),np.uint8),iterations=1).astype(bool)
        return kept_faces,kept_hands,[poses[i] for i in selected],mask,dict(
            detected_people=len(poses),raised_people=len(selected),selected_pose_indices=selected)

    def apply_raised_mask(packet, image, mask, info):
        depth = packet["depth"]
        grid_mask = cv2.resize(mask.astype(np.uint8),(depth.shape[1],depth.shape[0]),
                              interpolation=cv2.INTER_NEAREST).astype(bool)
        if "foreground_mask" in packet:
            grid_mask &= packet["foreground_mask"]
            packet["foreground_mask"] = grid_mask
        points = packet["points"]
        if len(points):
            K = packet["intrinsics"]
            x = np.clip(np.rint(points[:,0]*K[0,0]/points[:,2]+K[0,2]).astype(int),0,depth.shape[1]-1)
            y = np.clip(np.rint(points[:,1]*K[1,1]/points[:,2]+K[1,2]).astype(int),0,depth.shape[0]-1)
            keep = grid_mask[y,x]
            packet["points"] = points[keep].copy()
            packet["colors"] = packet["colors"][keep].copy()
        image = image.copy()
        image[~mask] = (24,24,24)
        depth_image = packet["depth_image"].copy()
        depth_image[~grid_mask] = 0
        packet["depth_image"] = depth_image
        packet["raised_person_mask"] = grid_mask
        packet["raised_hand_filter"] = dict(info,enabled=True,kept_points=len(packet["points"]))
        if packet.get("foreground",{}).get("enabled"):
            packet["foreground"] = dict(packet["foreground"],kept_pixels=int(grid_mask.sum()),
                                         kept_points=len(packet["points"]))
            foreground_status.content = f"**Foreground:** {len(packet['points']):,} points | Raised-hand people only"
        raised_hand_status.content = (
            f"**Raised-hand filter:** {info['raised_people']}/{info['detected_people']} people selected. "
            "Either wrist must be above its shoulder.")
        return packet,image

    def retain_gesture_hands(packet, detections, controls):
        # Visual filtering does not remove detections used by pan and zoom.
        packet["display_hands"] = packet.get("hands",[])
        packet["hands"] = [dict(landmarks=h[4:67].reshape(21,3).copy(),
                                right_hand_score=float(h[-2]),confidence=float(h[-1]))
                           for h in detections if h[-1]>=float(controls.get("hands_confidence",0.8))]
        packet["hand_detection_count"] = len(packet["hands"])
        return packet


    return (
        apply_raised_mask,
        raised_hand_status,
        retain_gesture_hands,
        select_raised_people,
    )


@app.cell
def processing(
    add_hands,
    add_poses,
    apply_raised_mask,
    canonical_face,
    cv2,
    depth_pipeline,
    detect_faces,
    extract_foreground,
    face_connections,
    face_live_status,
    face_overlay,
    face_pose_indices,
    gaze_calibration_status,
    hand_clock,
    hand_model,
    np,
    pose_model,
    raised_hand_status,
    retain_gesture_hands,
    select_raised_people,
    viewer,
    workbench,
):
    def process(frame, state, controls):
        packet = depth_pipeline(frame, state, controls)
        # Detect on the full undistorted image from this same source frame.
        image = cv2.remap(frame, *depth_pipeline.maps, interpolation=cv2.INTER_LINEAR)
        faces = detect_faces(image)
        hand_started = hand_clock.perf_counter()
        hands = hand_model(image) if controls.get("hands_enabled",True) else []
        packet["hand_inference_ms"] = (hand_clock.perf_counter()-hand_started)*1000
        pose_started = hand_clock.perf_counter()
        poses = pose_model(image) if (controls.get("pose_enabled",True) or controls.get("raised_hand_only",False)) else []
        packet["pose_inference_ms"] = (hand_clock.perf_counter()-pose_started)*1000
        all_hands = hands
        if controls.get("raised_hand_only",False):
            faces,hands,poses,raised_mask,raised_info = select_raised_people(
                faces,hands,poses,(image.shape[1],image.shape[0]),controls)
            if not faces:
                state.pop("foreground_face_box",None)
        else:
            raised_mask = None
            packet["raised_hand_filter"] = dict(enabled=False)
            raised_hand_status.content = "**Raised-hand filter: off**"
        surface_started = hand_clock.perf_counter()
        packet,image = extract_foreground(packet,image,faces,hands,state,controls)
        if raised_mask is not None:
            packet,image = apply_raised_mask(packet,image,raised_mask,raised_info)
        packet["foreground_ms"] = (hand_clock.perf_counter()-surface_started)*1000
        height, width = image.shape[:2]
        depth = packet["depth"]
        dh, dw = depth.shape
        K = depth_pipeline.calibration
        invK = np.linalg.inv(K)
        mesh_points, mesh_lines, gaze_lines, head_lines, records = [], [], [], [], []
        calibration_request = controls.get("gaze_calibration", {"request": 0})
        request_id = calibration_request.get("request", 0)
        if calibration_request.get("reset"):
            state.pop("eye_gaze_corrections", None)
            state["eye_gaze_request"] = request_id
        calibrating = request_id != state.get("eye_gaze_request", 0)

        def rotation_between(a, b):
            a, b = a/np.linalg.norm(a), b/np.linalg.norm(b)
            axis = np.cross(a,b)
            sine, cosine = np.linalg.norm(axis), float(np.clip(np.dot(a,b),-1,1))
            if sine < 1e-6:
                return np.eye(3)
            return cv2.Rodrigues(axis/sine*np.arctan2(sine,cosine))[0]

        def sample_depth(uv):
            x = np.clip(np.rint(uv[:,0]*dw/width).astype(int), 0, dw-1)
            y = np.clip(np.rint(uv[:,1]*dh/height).astype(int), 0, dh-1)
            return depth[y,x]

        def arrow(origin, direction, length):
            end = origin + length*direction
            side = np.cross(direction, [0.,1.,0.])
            if np.linalg.norm(side) < 0.01:
                side = np.cross(direction, [1.,0.,0.])
            side /= np.linalg.norm(side)
            up = np.cross(direction, side)
            return [[origin,end]] + [[end,end-length*0.15*direction+length*0.06*v]
                                    for v in [side,-side,up,-up]]

        def draw_direction(origin, direction, length, color):
            endpoints = np.array([origin, origin+length*direction])
            if np.all(endpoints[:,2] > 0.01):
                uv = endpoints @ K.T
                uv = np.clip(uv[:,:2]/uv[:,2,None], -10000, 10000).astype(int)
                cv2.arrowedLine(image, tuple(uv[0]), tuple(uv[1]), color, 2, cv2.LINE_AA, tipLength=0.15)

        for face in faces:
            landmarks = np.array(face, dtype=np.float64)
            uv = landmarks[:,:2]*[width,height]
            pixels = np.rint(uv).astype(int)
            inside = (uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
            lo = np.clip(pixels.min(axis=0), [0,0], [width-1,height-1])
            hi = np.clip(pixels.max(axis=0), [0,0], [width-1,height-1])
            cv2.rectangle(image, tuple(lo), tuple(hi), (80,255,120), 1)
            if controls.get("face_landmarks", True):
                for a,b in face_connections:
                    if inside[a] and inside[b]:
                        cv2.line(image, tuple(pixels[a]), tuple(pixels[b]), (80,255,120), 1, cv2.LINE_AA)
                for p in pixels[inside]:
                    cv2.circle(image, tuple(p), 1, (80,255,120), -1)
                for start in (468,473):
                    cv2.polylines(image, [pixels[start+1:start+5]], True, (0,175,255), 1, cv2.LINE_AA)
                z = sample_depth(uv)
                valid = inside & np.isfinite(z) & (z>0)
                xyz = (np.c_[uv,np.ones(len(uv))] @ invK.T)*z[:,None]
                # Lift surface annotations slightly toward the camera for visibility.
                xyz *= 0.997
                mesh_points.extend(xyz[valid])
                edges = face_connections[valid[face_connections].all(axis=1)]
                mesh_lines.extend(xyz[edges])

            record = {"box": [lo.tolist(),hi.tolist()], "landmarks": uv.copy(), "eyes": []}
            records.append(record)
            if not controls.get("gaze_enabled", True) or hi[0]-lo[0] < 40:
                continue
            ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                canonical_face[face_pose_indices], uv[face_pose_indices], K, None,
                iterationsCount=60, reprojectionError=5., flags=cv2.SOLVEPNP_EPNP)
            if not ok or inliers is None or len(inliers)<8:
                continue
            ids = inliers.ravel()
            rvec,tvec = cv2.solvePnPRefineLM(canonical_face[face_pose_indices[ids]],
                                           uv[face_pose_indices[ids]], K, None, rvec,tvec)
            R = cv2.Rodrigues(rvec)[0]
            translation = tvec.ravel()
            projected = cv2.projectPoints(canonical_face[face_pose_indices], rvec,tvec,K,None)[0][:,0]
            error = float(np.median(np.linalg.norm(projected-uv[face_pose_indices],axis=1)))
            nose_z = (R @ canonical_face[1] + translation)[2]
            nz = float(sample_depth(uv[[1]])[0])
            if error>6 or nose_z<=0 or not np.isfinite(nz) or nz<=0:
                continue
            scale = nz/nose_z
            head_direction = R @ np.array([0.,0.,-1.])
            face_width = np.linalg.norm(canonical_face[234]-canonical_face[454])*scale
            length = float(controls.get("gaze_length",3.0))*face_width
            record.update(head_direction=head_direction.copy(), pose_error_px=error)
            for iris, corners, lids in [(468,(33,133),(159,145)),(473,(362,263),(386,374))]:
                eye_width = np.linalg.norm(uv[corners[0]]-uv[corners[1]])
                opening = np.linalg.norm(uv[lids[0]]-uv[lids[1]])/max(eye_width,1.)
                if eye_width<9 or opening<0.12 or not inside[iris]:
                    continue
                # Assumed 12 mm eyeball radius and 9 mm depth behind eye corners.
                center_local = canonical_face[list(corners)].mean(axis=0) + [0.,0.,0.9]
                center = R @ center_local + translation
                ray = invK @ np.r_[uv[iris],1.]
                ray /= np.linalg.norm(ray)
                along = np.dot(ray,center)
                discriminant = 1.2**2 - np.dot(center-along*ray,center-along*ray)
                if discriminant<=0:
                    continue
                surface = (along-np.sqrt(discriminant))*ray
                direction = (surface-center)/1.2
                origin = surface*scale
                record["eyes"].append(dict(origin=origin.copy(), direction=direction.copy(),
                                           iris_index=iris, iris_pixel=uv[iris].copy(), opening=float(opening)))
            if calibrating and len(faces)==1 and len(record["eyes"])==2:
                # Personal neutral offsets are stored in head coordinates.
                state["eye_gaze_corrections"] = {
                    eye["iris_index"]: rotation_between(R.T@eye["direction"], R.T@(-eye["origin"]))
                    for eye in record["eyes"]}
                state["eye_gaze_request"] = request_id
                calibrating = False
            corrections = state.get("eye_gaze_corrections", {}) if len(faces)==1 else {}
            for eye in record["eyes"]:
                correction = corrections.get(eye["iris_index"], np.eye(3))
                direction = R @ correction @ R.T @ eye["direction"]
                eye["direction"] = direction.copy()
                origin = eye["origin"]
                gaze_lines.extend(arrow(origin,direction,length))
                head_lines.extend(arrow(origin,head_direction,length*0.65))
                # Keep projected endpoints in front of the camera.
                image_length = min(length,0.7*origin[2]/max(-direction[2],0.01))
                head_length = min(length*0.65,0.7*origin[2]/max(-head_direction[2],0.01))
                draw_direction(origin,head_direction,head_length,(255,220,20))
                draw_direction(origin,direction,image_length,(20,175,255))

        with viewer.server.atomic():
            for name, data in [("landmarks",mesh_points),("contours",mesh_lines),
                               ("gaze",gaze_lines),("head",head_lines)]:
                handle = face_overlay[name]
                handle.visible = bool(data)
                if data:
                    if name=="landmarks":
                        handle.points = np.asarray(data,dtype=np.float32)
                        handle.colors = np.tile(np.array([[80,255,120]],np.uint8),(len(data),1))
                    else:
                        handle.points = np.asarray(data,dtype=np.float32)
        eye_count = sum(len(r["eyes"]) for r in records)
        cv2.putText(image, f"Faces: {len(faces)}  Gaze rays: {eye_count} (approx.)", (8,20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (20,175,255), 1, cv2.LINE_AA)
        face_live_status.content = f"**Faces:** {len(faces)} | **Landmarks:** {sum(len(f) for f in faces)} | **Gaze rays:** {eye_count}"
        gaze_calibration_status.content = (
            "Waiting for one face with two open eyes. Look at the camera lens." if calibrating else
            "Neutral gaze offset applied for one visible face. Reset when the person changes. Accuracy is unvalidated."
            if state.get("eye_gaze_corrections") else
            "Gaze is uncalibrated. Look at the camera lens and press **Calibrate gaze** to correct the neutral offset.")
        packet.update(image=image, raw_image=frame, faces=records, face_count=len(faces), gaze_eye_count=eye_count,
                      gaze_neutral_calibrated=bool(state.get("eye_gaze_corrections")) and len(faces)==1,
                      gaze_kind="Uncalibrated iris/eyeball geometry estimate; relative depth anchor")
        packet = add_poses(add_hands(packet,hands,controls),
                           poses if controls.get("pose_enabled",True) else [],controls)
        return retain_gesture_hands(packet,all_hands,controls)

    workbench.set_processor(process)

    return


@app.cell
def rain_native_kernel(np):
    from cppyy_kit import cpp as rain_cpp
    from cppyy_kit.numpy_types import ConstNDArray as RainConstArray, NDArray as RainArray


    @rain_cpp(nogil=True)
    def rain_step(particles: RainArray[np.float32], depth: RainConstArray[np.float32],
                  mask: RainConstArray[np.uint8], lines: RainArray[np.float32],
                  colors: RainArray[np.uint8], rng: RainArray[np.uint32],
                  count: int, width: int, height: int,
                  fx: float, fy: float, cx: float, cy: float,
                  emitter_x: float, emitter_y: float, emitter_z: float,
                  head_width: float, head_top: float, dt: float,
                  gravity: float, speed: float) -> int:
        """
        // Particle rows: x, y, z, vx, vy, vz, remaining life, splash flag.
        // Camera axes: x right, y down, z forward. All distances are relative.
        if (width < 3 || height < 3 || depth_size != width*height ||
            mask_size != depth_size || rng_size < 1 || head_width <= 0 ||
            fx <= 0 || fy <= 0 || dt <= 0 || dt > 0.05) return -1;
        if (count < 0 || count*8 > particles_size || count*6 > lines_size ||
            count*6 > colors_size) return -2;
        auto random = [&]() {
            rng[0] = 1664525u*rng[0] + 1013904223u;
            return float(rng[0] >> 8) / 16777216.0f;
        };
        auto sample = [&](float x, float y, float z, int& u, int& v) {
            if (!(z > 0.001f)) return false;
            u = int(std::floor(fx*x/z+cx+0.5f));
            v = int(std::floor(fy*y/z+cy+0.5f));
            return u >= 1 && u < width-1 && v >= 1 && v < height-1 &&
                   mask[v*width+u] && depth[v*width+u] > 0;
        };
        int hits = 0;
        for (int i=0; i<count; ++i) {
            float* p = particles+8*i;
            p[6] -= dt;
            if (!(p[6] > 0) || !(p[2] > 0.001f) || p[1] > head_top+3*head_width) {
                p[0] = emitter_x+(random()-0.5f)*1.5f*head_width;
                p[1] = emitter_y-random()*1.2f*head_width;
                p[2] = emitter_z+(random()-0.5f)*0.8f*head_width;
                p[3] = (random()-0.5f)*0.12f*head_width;
                p[4] = speed*head_width*(0.8f+0.4f*random());
                p[5] = 0;
                p[6] = 3.0f;
                p[7] = 0;
            }
            const float old_x=p[0], old_y=p[1], old_z=p[2];
            p[4] += gravity*head_width*dt;
            p[0] += p[3]*dt; p[1] += p[4]*dt; p[2] += p[5]*dt;
            int u=0, v=0, pu=0, pv=0;
            if (p[7] < 0.5f && sample(p[0],p[1],p[2],u,v)) {
                const int j=v*width+u;
                const float surface=depth[j];
                const bool behind = p[2] >= surface;
                const bool was_behind = sample(old_x,old_y,old_z,pu,pv) && old_z >= depth[pv*width+pu];
                if (behind && !was_behind && p[2]-surface < head_width) {
                    // Surface normal from neighbouring unprojected depth samples.
                    // At the top silhouette, use an upward normal: the missing
                    // rear surface cannot be recovered from this single view.
                    float nx=0, ny=-1, nz=0;
                    const float dr=depth[j+1], dd=depth[j+width];
                    if (mask[j-width] && mask[j+1] && mask[j+width] && dr>0 && dd>0 &&
                        std::abs(dr-surface)<0.2f*head_width && std::abs(dd-surface)<0.2f*head_width) {
                        const float ax=((u+1-cx)*dr-(u-cx)*surface)/fx;
                        const float ay=(v-cy)*(dr-surface)/fy, az=dr-surface;
                        const float bx=(u-cx)*(dd-surface)/fx;
                        const float by=((v+1-cy)*dd-(v-cy)*surface)/fy, bz=dd-surface;
                        nx=ay*bz-az*by; ny=az*bx-ax*bz; nz=ax*by-ay*bx;
                        const float norm=std::sqrt(nx*nx+ny*ny+nz*nz);
                        if (norm>1e-9f) { nx/=norm; ny/=norm; nz/=norm; }
                        else { nx=0; ny=-1; nz=0; }
                    }
                    float vn=p[3]*nx+p[4]*ny+p[5]*nz;
                    if (vn>0) { nx=-nx; ny=-ny; nz=-nz; vn=-vn; }
                    p[0]=(u-cx)*surface/fx+nx*head_width*0.015f;
                    p[1]=(v-cy)*surface/fy+ny*head_width*0.015f;
                    p[2]=surface+nz*head_width*0.015f;
                    p[3]-=1.4f*vn*nx; p[4]-=1.4f*vn*ny; p[5]-=1.4f*vn*nz;
                    p[3]+=(random()-0.5f)*head_width*2.0f;
                    p[5]+=(random()-0.5f)*head_width;
                    p[6]=0.18f+random()*0.22f; p[7]=1;
                    ++hits;
                }
            }
            const float trail = p[7]>0.5f ? 0.012f : 0.025f;
            for (int axis=0; axis<3; ++axis) {
                lines[6*i+axis]=p[axis];
                lines[6*i+3+axis]=p[axis]-p[3+axis]*trail;
            }
            for (int end=0; end<2; ++end) {
                colors[6*i+3*end]=p[7]>0.5f ? 220 : 65;
                colors[6*i+3*end+1]=p[7]>0.5f ? 245 : 170;
                colors[6*i+3*end+2]=255;
            }
        }
        return hits;
        """


    return (rain_step,)


@app.cell
def rain_simulation(
    foreground_help,
    foreground_status,
    hand_clock,
    np,
    rain_step,
    viewer,
    workbench,
):
    import threading as rain_threading

    workbench.configure(rain_enabled=True, rain_count=800, rain_gravity=5.0, rain_speed=2.0)
    rain_particles = np.zeros((4096,8),np.float32)
    rain_lines = np.zeros((4096,2,3),np.float32)
    rain_colors = np.zeros((4096,2,3),np.uint8)
    rain_rng = np.array([728391],np.uint32)
    rain_lock = rain_threading.Lock()
    rain_stop = rain_threading.Event()
    rain_view = viewer.server.scene.add_line_segments(
        "/rain/drops",points=np.zeros((1,2,3),np.float32),colors=(65,170,255),
        thickness=1.8,thickness_units="screen",visible=False)
    rain_status = viewer.server.gui.add_markdown("**Rain:** waiting for a visible face.")
    rain_status.order = foreground_status.order - 0.02
    rain_help = viewer.server.gui.add_markdown(
        "**Rain:** blue falling drops and pale splashes. Gravity, particle integration, "
        "depth collisions and bounce run in the notebook's **cppyy_kit C++ kernel**. "
        "Toggle **rain_enabled**; adjust **rain_count**, **rain_speed** or **rain_gravity**. "
        "The surface and gravity scale are approximate.")
    rain_help.order = foreground_help.order + 0.001
    rain_metrics = dict(error=None,steps=0,collisions=0,publications=0,
                        kernel_ms=0.,source_sequence=None,source_stamp=None,head=None)

    def rain_head(packet):
        faces = packet.get("faces",[])
        if not faces:
            return None
        face = max(faces,key=lambda f:np.prod(np.asarray(f["box"])[1]-np.asarray(f["box"])[0]))
        box = np.asarray(face["box"],dtype=float)
        h,w = packet["image"].shape[:2]
        depth = packet["depth"]
        dh,dw = depth.shape
        x0,y0 = box[0]*[dw/w,dh/h]
        x1,y1 = box[1]*[dw/w,dh/h]
        left,right = np.clip(np.rint([x0+(x1-x0)*.25,x0+(x1-x0)*.75]).astype(int),0,dw-1)
        top,bottom = np.clip(np.rint([y0,y0+(y1-y0)*.35]).astype(int),0,dh-1)
        samples = depth[top:bottom+1,left:right+1]
        samples = samples[np.isfinite(samples)&(samples>0)]
        if len(samples)<4:
            return None
        z = float(np.median(samples))
        K = packet["intrinsics"]
        head_width = max(float((x1-x0)*z/K[0,0]),0.02)
        head_x = float(((x0+x1)/2-K[0,2])*z/K[0,0])
        head_top = float((y0-.12*(y1-y0)-K[1,2])*z/K[1,1])
        return np.array([head_x,head_top-1.6*head_width,z+.15*head_width,head_width,head_top])

    def run_rain():
        last_time = hand_clock.monotonic()
        last_publication = 0.
        last_status = 0.
        last_sequence = None
        head = None
        depth = mask = K = None
        while not rain_stop.wait(1/60):
            now = hand_clock.monotonic()
            dt = min(max(now-last_time,0.001),0.04)
            last_time = now
            try:
                packet = workbench.latest(processed=True)
                controls = workbench.controls()
                fresh = packet is not None and now-packet["stamp"]<0.5
                if not controls.get("rain_enabled",True) or not fresh:
                    rain_view.visible = False
                    rain_status.content = "**Rain:** paused" if fresh else "**Rain:** waiting for fresh depth"
                    continue
                if packet["sequence"]!=last_sequence:
                    last_sequence = packet["sequence"]
                    current_head = rain_head(packet)
                    head = current_head if head is None or current_head is None else .65*current_head+.35*head
                    depth = np.require(packet["depth"],dtype=np.float32,requirements=["C"])
                    source_mask = packet.get("foreground_mask",np.isfinite(depth)&(depth>0))
                    mask = np.require(source_mask,dtype=np.uint8,requirements=["C"])
                    K = packet["intrinsics"]
                    rain_metrics.update(source_sequence=last_sequence,source_stamp=packet["stamp"],
                                        head=None if head is None else head.tolist())
                if head is None or K is None:
                    rain_view.visible = False
                    rain_status.content = "**Rain:** waiting for a visible face"
                    continue
                count = int(np.clip(controls.get("rain_count",800),64,len(rain_particles)))
                start = hand_clock.perf_counter()
                with rain_lock:
                    hits = rain_step(rain_particles,depth,mask,rain_lines,rain_colors,rain_rng,
                                     count,depth.shape[1],depth.shape[0],
                                     float(K[0,0]),float(K[1,1]),float(K[0,2]),float(K[1,2]),
                                     *[float(v) for v in head],dt,
                                     float(np.clip(controls.get("rain_gravity",5.0),0,30)),
                                     float(np.clip(controls.get("rain_speed",2.0),0.1,10)))
                    elapsed = (hand_clock.perf_counter()-start)*1000
                    if hits<0:
                        raise RuntimeError(f"C++ rain kernel rejected buffers: {hits}")
                    if now-last_publication>=1/30:
                        # Publish owned copies. The next native step mutates only
                        # simulation buffers, never arrays queued for the browser.
                        with viewer.server.atomic():
                            rain_view.points = rain_lines[:count].copy()
                            rain_view.colors = rain_colors[:count].copy()
                            rain_view.visible = True
                        last_publication = now
                        rain_metrics["publications"] += 1
                rain_metrics.update(error=None,steps=rain_metrics["steps"]+1,
                                    collisions=rain_metrics["collisions"]+hits,kernel_ms=elapsed)
                if now-last_status>.25:
                    rain_status.content = (
                        f"**Rain:** {count:,} drops | Splashes: {rain_metrics['collisions']:,} | "
                        f"C++ step: {elapsed:.2f} ms")
                    last_status = now
            except Exception as exc:
                rain_metrics["error"] = f"{type(exc).__name__}: {exc}"
                rain_view.visible = False
                rain_status.content = f"**Rain error:** {rain_metrics['error']}"
        rain_view.visible = False

    rain_thread = rain_threading.Thread(target=run_rain,name="native-rain",daemon=True)
    rain_thread.start()

    return


@app.cell
def display(workbench):
    viewer = ViserViewer(workbench, port=8768)
    # Alternative: close viewer first, then replace the line above with
    # viewer = PolyscopeViewer(workbench)
    return (viewer,)


@app.cell
def face_overlay_view(np, viewer, workbench):
    face_overlay = {
        "landmarks": viewer.server.scene.add_point_cloud(
            "/faces/landmarks", points=np.zeros((1,3), np.float32),
            colors=np.array([[80,255,120]], np.uint8), point_size=0.006, visible=False),
        "contours": viewer.server.scene.add_line_segments(
            "/faces/contours", points=np.zeros((1,2,3), np.float32),
            colors=(80,255,120), line_width=2, visible=False),
        "gaze": viewer.server.scene.add_line_segments(
            "/faces/gaze", points=np.zeros((1,2,3), np.float32),
            colors=(255,175,20), line_width=5, visible=False),
        "head": viewer.server.scene.add_line_segments(
            "/faces/head_direction", points=np.zeros((1,2,3), np.float32),
            colors=(20,220,255), line_width=3, visible=False),
    }
    workbench.configure(face_landmarks=True, gaze_enabled=True, gaze_length=3.0)
    face_legend = viewer.server.gui.add_markdown(
        "**Faces:** green landmarks. **Orange:** approximate iris gaze. "
        "**Cyan:** head direction. Gaze uses an assumed eyeball geometry, "
        "without personal calibration. Cloud positions and lengths have relative scale.")
    face_live_status = viewer.server.gui.add_markdown("Waiting for face landmarks.")

    return face_legend, face_live_status, face_overlay


@app.cell
def gaze_controls(viewer, workbench):
    workbench.configure(gaze_calibration={"request": 0, "reset": False})
    gaze_calibrate_button = viewer.server.gui.add_button("Calibrate gaze: look at camera")
    gaze_reset_button = viewer.server.gui.add_button("Reset gaze calibration")
    gaze_calibration_status = viewer.server.gui.add_markdown(
        "For one visible face: look directly at the camera lens, then press **Calibrate gaze**. "
        "This corrects a neutral offset. It does not validate gaze accuracy.")

    def request_gaze_calibration(event):
        if event.client is not None:
            request = workbench.controls().get("gaze_calibration", {}).get("request", 0) + 1
            workbench.configure(gaze_calibration={"request": request, "reset": False})

    def reset_gaze_calibration(event):
        if event.client is not None:
            request = workbench.controls().get("gaze_calibration", {}).get("request", 0) + 1
            workbench.configure(gaze_calibration={"request": request, "reset": True})

    gaze_calibrate_button.on_click(request_gaze_calibration)
    gaze_reset_button.on_click(reset_gaze_calibration)

    return gaze_calibrate_button, gaze_calibration_status, gaze_reset_button


@app.cell
def face_controls_layout(
    face_legend,
    face_live_status,
    gaze_calibrate_button,
    gaze_calibration_status,
    gaze_reset_button,
    viewer,
):
    # Keep face status and calibration beside the live previews.
    face_live_status.order = viewer.depth_image.order + 0.1
    gaze_calibrate_button.order = viewer.depth_image.order + 0.2
    gaze_reset_button.order = viewer.depth_image.order + 0.3
    gaze_calibration_status.order = viewer.depth_image.order + 0.4
    face_legend.order = viewer.depth_image.order + 0.5

    face_legend.content = (
        "**Green:** face landmarks. **Orange:** approximate iris gaze. "
        "**Cyan:** head direction. Gaze uses assumed eyeball geometry with an optional "
        "neutral offset. Accuracy is unvalidated. Scene scale is relative.")

    return


@app.cell
def hand_overlay_view(
    cv2,
    depth_pipeline,
    face_legend,
    face_live_status,
    hand_connections,
    np,
    viewer,
    workbench,
):
    hand_overlay = {
        "joints": viewer.server.scene.add_point_cloud(
            "/hands/joints", points=np.zeros((1,3),np.float32),
            colors=np.array([[255,70,210]],np.uint8), point_size=0.01, visible=False),
        "bones": viewer.server.scene.add_line_segments(
            "/hands/skeleton", points=np.zeros((1,2,3),np.float32),
            colors=(255,70,210), line_width=4, visible=False),
    }
    workbench.configure(hands_enabled=True, hands_3d=True, hands_confidence=0.8)
    hand_live_status = viewer.server.gui.add_markdown("Waiting for hands.")
    hand_live_status.order = face_live_status.order + 0.01
    hand_legend = viewer.server.gui.add_markdown(
        "**Magenta:** hand boxes and 21 joints per hand. The 3D skeleton combines "
        "model joint depth with the palm's estimated scene depth. Its scale is relative.")
    hand_legend.order = face_legend.order + 0.01

    def add_hands(packet, detections, controls):
        image = packet["image"]
        height,width = image.shape[:2]
        depth = packet["depth"]
        dh,dw = depth.shape
        K = depth_pipeline.calibration
        invK = np.linalg.inv(K)
        joints, bones, records = [], [], []
        for hand in detections:
            confidence = float(hand[-1])
            if confidence < float(controls.get("hands_confidence",0.8)):
                continue
            landmarks = hand[4:67].reshape(21,3).copy()
            uv = landmarks[:,:2]
            pixels = np.rint(uv).astype(int)
            inside = (uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
            box = np.clip(hand[:4].reshape(2,2), [0,0], [width-1,height-1]).astype(int)
            cv2.rectangle(image,tuple(box[0]),tuple(box[1]),(210,70,255),2)
            cv2.putText(image,f"Hand {len(records)+1} {confidence:.2f}",
                        (int(box[0,0]),max(34,int(box[0,1])-5)),
                        cv2.FONT_HERSHEY_SIMPLEX,0.45,(210,70,255),1,cv2.LINE_AA)
            for a,b in hand_connections:
                if inside[a] and inside[b]:
                    cv2.line(image,tuple(pixels[a]),tuple(pixels[b]),(210,70,255),2,cv2.LINE_AA)
            for point in pixels[inside]:
                cv2.circle(image,tuple(point),3,(255,230,255),-1,cv2.LINE_AA)
            record = dict(box=box.copy(), confidence=confidence,
                          landmarks=landmarks, right_hand_score=float(hand[-2]))
            records.append(record)
            if not controls.get("hands_3d",True):
                continue
            # Anchor the palm to DA3. Use model-relative joint depth to avoid
            # sampling background through gaps between thin fingers.
            palm_ids = np.array([0,5,9,13,17])
            palm_ids = palm_ids[inside[palm_ids]]
            if len(palm_ids)<3:
                continue
            x = np.clip(np.rint(uv[palm_ids,0]*dw/width).astype(int),0,dw-1)
            y = np.clip(np.rint(uv[palm_ids,1]*dh/height).astype(int),0,dh-1)
            sampled = depth[y,x]
            valid = np.isfinite(sampled)&(sampled>0)
            if valid.sum()<3:
                continue
            anchor = float(np.median(sampled[valid]))
            model_offset = landmarks[:,2]-np.median(landmarks[palm_ids[valid],2])
            z = anchor+model_offset*anchor/float((K[0,0]+K[1,1])/2)
            valid_joints = inside & np.isfinite(z) & (z>0)
            xyz = (np.c_[uv,np.ones(21)]@invK.T)*z[:,None]
            record["points"] = xyz.copy()
            joints.extend(xyz[valid_joints])
            edges = hand_connections[valid_joints[hand_connections].all(axis=1)]
            bones.extend(xyz[edges])
        with viewer.server.atomic():
            hand_overlay["joints"].visible = bool(joints)
            hand_overlay["bones"].visible = bool(bones)
            if joints:
                hand_overlay["joints"].points = np.asarray(joints,np.float32)
                hand_overlay["joints"].colors = np.tile(np.array([[255,70,210]],np.uint8),(len(joints),1))
            if bones:
                hand_overlay["bones"].points = np.asarray(bones,np.float32)
        hand_live_status.content = f"**Hands:** {len(records)} | **Hand landmarks:** {21*len(records)}"
        cv2.putText(image,f"Hands: {len(records)}",(8,40),cv2.FONT_HERSHEY_SIMPLEX,0.48,(210,70,255),1,cv2.LINE_AA)
        packet.update(hands=records, hand_count=len(records),
                      hand_depth_kind="Palm-anchored model skeleton; relative scale")
        return packet


    return add_hands, hand_legend, hand_live_status


@app.cell
def pose_overlay_view(
    cv2,
    depth_pipeline,
    hand_legend,
    hand_live_status,
    np,
    pose_connections,
    viewer,
    workbench,
):
    pose_overlay = {
        "joints": viewer.server.scene.add_point_cloud(
            "/body/joints",points=np.zeros((1,3),np.float32),
            colors=np.array([[255,245,40]],np.uint8),point_size=0.012,visible=False),
        "bones": viewer.server.scene.add_line_segments(
            "/body/skeleton",points=np.zeros((1,2,3),np.float32),
            colors=(255,245,40),line_width=4,visible=False),
    }
    workbench.configure(pose_enabled=True, pose_3d=True, pose_confidence=0.5)
    pose_live_status = viewer.server.gui.add_markdown("Waiting for body pose.")
    pose_live_status.order = hand_live_status.order + 0.005
    pose_legend = viewer.server.gui.add_markdown(
        "**Yellow:** body pose, up to 33 joints per person. Hidden or out-of-frame joints "
        "are omitted. The 3D skeleton uses model joint depths anchored to estimated torso "
        "depth. Scene scale is relative.")
    pose_legend.order = hand_legend.order + 0.005

    def add_poses(packet, detections, controls):
        image = packet["image"]
        height,width = image.shape[:2]
        depth = packet["depth"]
        dh,dw = depth.shape
        K = depth_pipeline.calibration
        invK = np.linalg.inv(K)
        threshold = float(controls.get("pose_confidence",0.5))
        joints,bones,records = [],[],[]
        for detection in detections:
            box,landmarks,world,mask,heatmap,confidence = detection
            if float(confidence)<threshold:
                continue
            landmarks = np.asarray(landmarks[:33],dtype=np.float64).copy()
            uv = landmarks[:,:2]
            valid = (np.isfinite(landmarks).all(axis=1)
                     & (landmarks[:,3]>=threshold) & (landmarks[:,4]>=threshold)
                     & (uv[:,0]>=0) & (uv[:,0]<width) & (uv[:,1]>=0) & (uv[:,1]<height))
            if valid.sum()<3:
                continue
            pixels = np.rint(uv).astype(int)
            edges = pose_connections[valid[pose_connections].all(axis=1)]
            for a,b in edges:
                cv2.line(image,tuple(pixels[a]),tuple(pixels[b]),(40,245,255),2,cv2.LINE_AA)
            for point in pixels[valid]:
                cv2.circle(image,tuple(point),4,(40,245,255),-1,cv2.LINE_AA)
            record = dict(confidence=float(confidence), landmarks=landmarks,
                          visible=valid.copy(), visible_joints=int(valid.sum()))
            records.append(record)
            if not controls.get("pose_3d",True):
                continue
            # Anchor visible torso joints to the aligned DA3 grid. Preserve the
            # model's relative joint depths rather than sampling background at limbs.
            anchors = np.array([11,12,23,24])
            anchors = anchors[valid[anchors]]
            if len(anchors)<2:
                anchors = np.flatnonzero(valid)
            x = np.clip(np.rint(uv[anchors,0]*dw/width).astype(int),0,dw-1)
            y = np.clip(np.rint(uv[anchors,1]*dh/height).astype(int),0,dh-1)
            samples = depth[y,x]
            usable = np.isfinite(samples)&(samples>0)
            if usable.sum()<2:
                continue
            anchor = float(np.median(samples[usable]))
            offset = landmarks[:,2]-np.median(landmarks[anchors[usable],2])
            z = anchor+offset*anchor/float((K[0,0]+K[1,1])/2)
            valid3d = valid & np.isfinite(z) & (z>0)
            xyz = (np.c_[uv,np.ones(33)]@invK.T)*z[:,None]
            record.update(points=xyz.copy(),visible_3d=valid3d.copy())
            joints.extend(xyz[valid3d])
            edges3d = pose_connections[valid3d[pose_connections].all(axis=1)]
            bones.extend(xyz[edges3d])
        with viewer.server.atomic():
            pose_overlay["joints"].visible = bool(joints)
            pose_overlay["bones"].visible = bool(bones)
            if joints:
                pose_overlay["joints"].points = np.asarray(joints,np.float32)
                pose_overlay["joints"].colors = np.tile(np.array([[255,245,40]],np.uint8),(len(joints),1))
            if bones:
                pose_overlay["bones"].points = np.asarray(bones,np.float32)
        visible_count = sum(r["visible_joints"] for r in records)
        pose_live_status.content = f"**Body poses:** {len(records)} | **Visible joints:** {visible_count}"
        cv2.putText(image,f"Body poses: {len(records)}",(8,60),cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,(40,245,255),1,cv2.LINE_AA)
        packet.update(poses=records,pose_count=len(records),pose_visible_joints=visible_count,
                      pose_depth_kind="Torso-anchored model skeleton; relative scale")
        return packet


    return (add_poses,)


@app.cell
def hand_drag_runtime(
    TolerantHandDrag,
    hand_clock,
    hand_legend,
    hand_live_status,
    viewer,
    workbench,
):
    import threading as drag_threading

    workbench.configure(hand_zoom=workbench.controls().get("hand_zoom",True),
                        hand_zoom_gain=workbench.controls().get("hand_zoom_gain",1.0))
    hand_drag_status = viewer.server.gui.add_markdown("**Hand grab: released**. One fist pans; two fists zoom.")
    hand_drag_status.order = hand_live_status.order + 0.01
    hand_drag_legend = viewer.server.gui.add_markdown(
        "**One fist:** move to pan. **Two fists:** spread apart to zoom in; bring together "
        "to zoom out. Opening releases immediately. Up to two missed detection frames "
        "retain the grab with movement paused; a third miss releases it. "
        "Use **hand_drag** to enable gestures, **hand_zoom** to enable two-hand zoom, "
        "and the gain controls to adjust sensitivity.")
    hand_drag_legend.order = hand_legend.order + 0.01
    hand_drag = TolerantHandDrag()
    hand_drag_stop = drag_threading.Event()
    hand_drag_runtime = {"error": None, "sequence": None, "stamp": None}

    def run_hand_drag():
        previous = None
        last_text = None
        while not hand_drag_stop.wait(0.02):
            try:
                packet = workbench.latest(processed=True)
                controls = workbench.controls()
                if not controls.get("hand_drag",True):
                    hand_drag.release("Hand drag disabled")
                elif packet is None or hand_clock.monotonic()-packet["stamp"]>0.5:
                    hand_drag.release("No fresh hand frame")
                elif workbench.status()["error"] is not None:
                    hand_drag.release("Processing error")
                elif packet["sequence"] != previous:
                    previous = packet["sequence"]
                    hand_drag_runtime.update(sequence=previous,stamp=packet["stamp"])
                    image = packet["image"]
                    hand_drag.update(packet.get("hands",[]),(image.shape[1],image.shape[0]),
                                     viewer.server.get_clients(),
                                     gain=float(controls.get("hand_drag_gain",0.6)),
                                     zoom_enabled=bool(controls.get("hand_zoom",True)),
                                     zoom_gain=float(controls.get("hand_zoom_gain",1.0)))
                hand_drag_runtime["error"] = None
                text = (f"**Hand grab: {hand_drag.mode.upper() if hand_drag.held else 'released'}** | "
                        f"Curled fingers: {hand_drag.curled}/4. Missed frames: {hand_drag.missed_frames}/2. {hand_drag.reason}")
                if text != last_text:
                    hand_drag_status.content = text
                    last_text = text
            except Exception as exc:
                hand_drag.release("Drag error")
                hand_drag_runtime["error"] = f"{type(exc).__name__}: {exc}"
                hand_drag_status.content = f"Hand drag released: {hand_drag_runtime['error']}"
        hand_drag.release("Hand drag stopped")

    hand_drag_thread = drag_threading.Thread(target=run_hand_drag,name="hand-drag",daemon=True)
    hand_drag_thread.start()

    return


@app.cell
def introduction():
    mo.md("""
    # Webcam → DA3 → depth and cloud

    Open http://127.0.0.1:8768 for RGB, depth and a colored cloud together.
    Depth is relative estimated geometry, not metres. Frames are undistorted
    using the measured webcam calibration. Point stride controls cloud density.

    Retain the model and camera cells while revising processing. Close the
    current viewer before replacing its display cell. Follow the complete worker
    shutdown sequence in `SESSION_HANDOFF.md` when done. It covers rain,
    gestures, the viewer, capture and the face subprocess.
    """)
    return


if __name__ == "__main__":
    app.run()
