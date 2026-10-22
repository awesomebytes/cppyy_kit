```bash
cd /home/sapf/playground/ros_project/cppyy_kit
./roscon_uk_2026/run_robot_demo.sh baseline --gazebo-gui
```

```bash
cd /home/sapf/playground/ros_project/cppyy_kit
./roscon_uk_2026/run_robot_demo.sh forward --gazebo-gui
```

```bash
cd /home/sapf/playground/ros_project/cppyy_kit
./roscon_uk_2026/run_robot_demo.sh admittance --gazebo-gui
```

```bash
cd /home/sapf/playground/ros_project/cppyy_kit
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/notebook.sh edit \
  roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/live_notebook.py \
  --host 127.0.0.1 --port 2721 --no-token --no-sandbox
```

```bash
xdg-open http://127.0.0.1:2721
xdg-open http://127.0.0.1:8768
```

```bash
cd /home/sapf/playground/ros_project/cppyy_kit
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/notebook.sh pair notebook list \
  --url http://127.0.0.1:2721
```

```bash
cd /home/sapf/playground/ros_project/cppyy_kit
roscon_uk_2026/next_steps/agent_session/notebook.sh edit \
  roscon_uk_2026/next_steps/agent_session/notebook.py \
  --host 127.0.0.1 --port 2718 --no-token --no-sandbox
```

```bash
xdg-open http://127.0.0.1:2718
```
