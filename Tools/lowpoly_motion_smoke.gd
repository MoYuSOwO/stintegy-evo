extends SceneTree

func _initialize() -> void:
    call_deferred("run")

func run() -> void:
    var scene = load("res://Levels/lowpoly.tscn").instantiate()
    root.add_child(scene)
    current_scene = scene
    var deadline := Time.get_ticks_msec() + 30000
    while not scene.get("IsInitialized") and Time.get_ticks_msec() < deadline:
        await process_frame
    if not scene.get("IsInitialized"):
        push_error("MOTION timeout: 3D scene did not initialize")
        quit(1)
        return

    # A fixed external command validates motion without introducing a concrete
    # controller or putting a driving algorithm in the presentation layer.
    scene.call("SetExternalInput", 0, 0.0, 2.0, 0.0)
    Engine.max_fps = 60
    var car: Node3D = scene.get_node("Car_01")
    var start_position := car.global_position
    var previous := start_position
    var simulation_start: float = scene.get("RaceSeconds")
    var moving_frames := 0
    var frame_count := 0
    var started := Time.get_ticks_usec()
    while Time.get_ticks_usec() - started < 4000000:
        await process_frame
        if previous.distance_to(car.global_position) > 0.0001:
            moving_frames += 1
        previous = car.global_position
        frame_count += 1

    var elapsed := (Time.get_ticks_usec() - started) / 1000000.0
    var race_delta: float = scene.get("RaceSeconds") - simulation_start
    var moved := start_position.distance_to(car.global_position)
    var failures: Array[String] = []
    if moved < 1.0:
        failures.append("Known external input did not move the car")
    if moving_frames < frame_count * 0.5:
        failures.append("Externally driven car did not produce enough moving frames")
    if race_delta < elapsed * 0.75 or race_delta > elapsed * 1.25:
        failures.append("Simulation cadence is outside the expected range")

    var result := {
        "input": {"desired_curvature": 0.0, "desired_accel": 2.0},
        "elapsed_s": elapsed,
        "race_delta_s": race_delta,
        "moved_m": moved,
        "frames": frame_count,
        "moving_frames": moving_frames
    }
    DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path("res://.tmp/lowpoly"))
    var file := FileAccess.open("res://.tmp/lowpoly/motion-report.json", FileAccess.WRITE)
    file.store_string(JSON.stringify([result], "  "))
    file.close()
    print("MOTION ", JSON.stringify(result))
    print("MOTION ", "PASS" if failures.is_empty() else "FAIL", " ", failures)
    quit(0 if failures.is_empty() else 1)
