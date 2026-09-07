extends SceneTree

func _initialize() -> void:
    call_deferred("run")

func run() -> void:
    var scene = load("res://Levels/lowpoly.tscn").instantiate()
    root.add_child(scene)
    current_scene = scene
    while scene.get_node_or_null("CameraRig/Lens") == null:
        await process_frame
    var car: Node3D = scene.get_node("Car_01")
    var camera: Camera3D = scene.get_node("CameraRig/Lens")
    var failures: Array[String] = []
    var report: Array = []
    for fps in [120, 60, 30]:
        Engine.max_fps = fps
        await create_timer(2.0).timeout
        var previous := car.global_position
        var speeds: Array[float] = []
        var still := 0
        var start := Time.get_ticks_usec()
        var last := start
        var simulation_start: float = scene.get("RaceSeconds")
        var screen_min := INF
        var screen_max := -INF
        while Time.get_ticks_usec() - start < 4000000:
            await process_frame
            var now := Time.get_ticks_usec()
            var dt := (now - last) / 1000000.0
            var distance := previous.distance_to(car.global_position)
            speeds.append(distance / max(dt, 0.00001))
            if distance < 0.0001:
                still += 1
            var screen := camera.unproject_position(car.global_position)
            screen_min = min(screen_min, screen.y)
            screen_max = max(screen_max, screen.y)
            previous = car.global_position
            last = now
        speeds.sort()
        var median := speeds[speeds.size() / 2]
        var p05 := speeds[int(speeds.size() * 0.05)]
        var p95 := speeds[int(speeds.size() * 0.95)]
        var ratio: float = (scene.get("RaceSeconds") - simulation_start) / ((last - start) / 1000000.0)
        var result := {"fps_cap": fps, "frames": speeds.size(), "still_frames": still, "p05_speed": p05, "median_speed": median, "p95_speed": p95, "simulation_rate": ratio, "screen_y_range": screen_max - screen_min}
        report.append(result)
        print("MOTION ", JSON.stringify(result))
        if still > speeds.size() * 0.05:
            failures.append("Repeated frozen frames at " + str(fps) + " FPS")
        if ratio < 0.9 or ratio > 1.1:
            failures.append("Simulation cadence is not real time at " + str(fps) + " FPS")
    DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path("res://.tmp/lowpoly"))
    var file := FileAccess.open("res://.tmp/lowpoly/motion-report.json", FileAccess.WRITE)
    file.store_string(JSON.stringify(report, "  "))
    print("MOTION ", "PASS" if failures.is_empty() else "FAIL", " ", failures)
    quit(0 if failures.is_empty() else 1)
