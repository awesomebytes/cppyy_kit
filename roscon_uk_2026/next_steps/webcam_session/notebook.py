import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium")

with app.setup:
    import cv2
    import numpy as np
    import cv_kit
    import marimo as mo
    from roscon_uk_2026.next_steps.webcam_session.live import LiveCamera


@app.cell
def camera_session():
    cv = cv_kit.bringup_cv()
    camera = LiveCamera(camera=0, port=8766)
    return (camera,)


@app.cell
def processing(camera):
    import json
    import sys
    from pathlib import Path

    _dlib_path = str(Path("build/webcam_session/dlib_python").resolve())
    if _dlib_path not in sys.path:
        sys.path.append(_dlib_path)
    import dlib

    # Model source: https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet
    face_detector = cv2.FaceDetectorYN.create(
        "roscon_uk_2026/next_steps/webcam_session/models/face_detection_yunet_2023mar.onnx",
        "", (320, 320), score_threshold=0.8
    )


    def detect_features(frame, detector):
        height, width = frame.shape[:2]
        scale = 320 / max(height, width)
        small = cv2.resize(frame, (round(width * scale), round(height * scale)))
        canvas = np.zeros((320, 320, 3), dtype=np.uint8)
        top, left = (320 - small.shape[0]) // 2, (320 - small.shape[1]) // 2
        canvas[top:top + small.shape[0], left:left + small.shape[1]] = small
        boxes, scores, points = [], [], []
        # Detect near upright in each view, then undo the rotation and padding.
        for angle in (0, -30, 30, -60, 60, -90, 90):
            rotation = cv2.getRotationMatrix2D((160, 160), angle, 1)
            view = cv2.warpAffine(canvas, rotation, (320, 320))
            _, faces = detector.detect(view)
            if faces is None:
                continue
            inverse = cv2.invertAffineTransform(rotation)
            for face in faces:
                landmarks = cv2.transform(face[4:14].reshape(1, 5, 2), inverse)[0]
                landmarks = (landmarks - [left, top]) / scale
                x, y, w, h = face[:4]
                corners = np.array([[[x, y], [x + w, y], [x + w, y + h], [x, y + h]]], dtype=np.float32)
                corners = (cv2.transform(corners, inverse)[0] - [left, top]) / scale
                lo, hi = corners.min(axis=0), corners.max(axis=0)
                boxes.append([float(lo[0]), float(lo[1]), float(hi[0] - lo[0]), float(hi[1] - lo[1])])
                scores.append(float(face[14]))
                points.append(landmarks)
        keep = cv2.dnn.NMSBoxes(boxes, scores, 0.8, 0.3)
        return [(boxes[int(i)], points[int(i)]) for i in np.asarray(keep).reshape(-1)]


    def process(frame, state, detector=face_detector):
        detections = detect_features(frame, detector)
        output = frame.copy()
        for box, landmarks in detections:
            x, y, w, h = np.rint(box).astype(int)
            cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2)
            for point, label, color in zip(
                landmarks,
                ("R eye", "L eye", "nose", "R mouth", "L mouth"),
                ((255, 200, 0), (255, 200, 0), (0, 255, 255), (255, 0, 255), (255, 0, 255))
            ):
                px, py = np.rint(point).astype(int)
                cv2.circle(output, (px, py), 4, color, -1)
                cv2.putText(output, label, (px + 6, py - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(output, label, (px + 6, py - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
        cv2.putText(output, f"Faces: {len(detections)} | rotation-aware landmarks", (16, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return output





    # Keep both processors available without recreating the camera.
    yunet_process = process
    dlib_detector = dlib.get_frontal_face_detector()
    dlib_predictor = dlib.shape_predictor(
        "roscon_uk_2026/next_steps/webcam_session/models/shape_predictor_68_face_landmarks.dat"
    )


    def dlib_process(frame, state, detector=dlib_detector, predictor=dlib_predictor):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detector(gray, 0)
        output = frame.copy()
        for face in faces:
            shape = predictor(gray, face)
            points = np.array([(p.x, p.y) for p in shape.parts()], dtype=np.int32)
            cv2.rectangle(output, (face.left(), face.top()),
                          (face.right(), face.bottom()), (0, 255, 0), 2)
            for start, end, closed, color in (
                (0, 17, False, (0, 220, 0)),
                (17, 22, False, (255, 200, 0)), (22, 27, False, (255, 200, 0)),
                (27, 31, False, (0, 255, 255)), (31, 36, False, (0, 255, 255)),
                (36, 42, True, (255, 200, 0)), (42, 48, True, (255, 200, 0)),
                (48, 60, True, (255, 0, 255)), (60, 68, True, (255, 0, 255)),
            ):
                cv2.polylines(output, [points[start:end]], closed, color, 1, cv2.LINE_AA)
                for point in points[start:end]:
                    cv2.circle(output, tuple(point), 2, color, -1, cv2.LINE_AA)
        cv2.putText(output, f"dlib: 68 landmarks | Faces: {len(faces)}", (16, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        return output


    processors = {"dlib": dlib_process, "yunet": yunet_process}
    processor_names = {"dlib": "dlib | 68 landmarks", "yunet": "YuNet | 5 landmarks, rotated views"}
    selected_processor = {"name": "dlib"}
    processor_viewer = b'''<!doctype html>
    <html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
    <title>Interactive webcam</title><style>
    body{margin:24px;background:#161616;color:#eee;font:16px system-ui}
    select{font:inherit;padding:8px;margin:0 12px 16px 8px;background:#292929;color:white;border:1px solid #777;border-radius:6px}
    img{display:block;max-width:95vw;max-height:80vh}#status{margin-top:12px;color:#ccc}
    </style><label for="processor">Face landmarks</label><select id="processor"></select>
    <span id="switch"></span><img id="frame" alt="Live webcam"><div id="status"></div>
    <script>
    const frame=document.getElementById('frame'),status=document.getElementById('status');
    const select=document.getElementById('processor'),message=document.getElementById('switch');
    let changing=false;
    select.onchange=async()=>{changing=true;select.disabled=true;try{
     const response=await fetch('/processor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name:select.value})});
     if(!response.ok)throw Error(await response.text());message.textContent='';
    }catch(e){message.textContent=String(e)}finally{changing=false;select.disabled=false}};
    async function update(){try{
     const response=await fetch('/frame.jpg');
     if(response.ok){const url=URL.createObjectURL(await response.blob());const old=frame.src;
     frame.src=url;if(old.startsWith('blob:'))URL.revokeObjectURL(old)}
     const [state,models]=await Promise.all([fetch('/status').then(r=>r.json()),fetch('/processors').then(r=>r.json())]);
     if(!select.options.length){for(const [value,label] of Object.entries(models.choices)){select.add(new Option(label,value))}}
     if(!changing)select.value=models.selected;
     status.textContent=state.error?'Processing error: '+state.error:
     `${models.choices[models.selected]} | ${state.processing_ms.toFixed(1)} ms processing | Camera running`;
    }catch(e){status.textContent=String(e)}setTimeout(update,100)}update();
    </script></html>'''


    # Replace only the request handler on the existing viewer server.
    if not hasattr(camera, "_base_viewer_handler"):
        camera._base_viewer_handler = camera._server.RequestHandlerClass


    class ProcessorHandler(camera._base_viewer_handler):
        def reply(self, body, content_type="application/json"):
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self.reply(processor_viewer, "text/html; charset=utf-8")
            elif self.path == "/processors":
                self.reply(json.dumps({"choices": processor_names, "selected": selected_processor["name"]}).encode())
            else:
                super().do_GET()

        def do_POST(self):
            if self.path != "/processor":
                self.send_error(404)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 1024 or self.headers.get_content_type() != "application/json":
                    raise ValueError("Expected a small JSON request")
                name = json.loads(self.rfile.read(length))["name"]
                function = processors[name]
            except (ValueError, KeyError, TypeError):
                self.send_error(400, "Unknown processor or invalid request")
                return
            camera.set_processor(function, label=processor_names[name])
            selected_processor["name"] = name
            self.reply(json.dumps({"selected": name}).encode())


    dlib_process(np.zeros((480, 640, 3), dtype=np.uint8), {})
    camera._server.RequestHandlerClass = ProcessorHandler
    camera.set_processor(dlib_process, label=processor_names["dlib"])

    return


@app.cell
def viewer(camera):
    mo.md(f"""
    # Interactive webcam

    [Open the continuous viewer in a separate window]({camera.url}).
    Edit the processing cell through marimo or the agent interface. The camera cell
    owns the persistent session. Inspect `camera.status()` and `camera.snapshot()`.
    Call `camera.close()` when finished.
    """)
    return


if __name__ == "__main__":
    app.run()
