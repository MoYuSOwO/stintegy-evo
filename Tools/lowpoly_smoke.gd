extends SceneTree

var failures: Array[String] = []
var scene: Node
var captures: Array[String] = []
var finished := false
const DEADLINE_MS := 90000

func _initialize() -> void:
    call_deferred("run")
    call_deferred("watchdog")

func watchdog() -> void:
    await create_timer(DEADLINE_MS / 1000.0).timeout
    if not finished:
        push_error("SMOKE timeout: scene or renderer did not complete")
        quit(1)

func check(ok: bool, message: String) -> void:
    if not ok:
        failures.append(message)
        push_error(message)

func press(code: Key) -> void:
    var event := InputEventKey.new()
    event.keycode = code
    event.pressed = true
    Input.parse_input_event(event)
    var released := InputEventKey.new()
    released.keycode = code
    Input.parse_input_event(released)

func strategy_readout() -> String:
    for label in scene.find_children("*", "Label", true, false):
        if "Q / E" in label.text:
            return label.text
    return ""

func capture(name: String) -> void:
    if DisplayServer.get_name() == "headless":
        print("CAPTURE SKIP (headless, not visual proof): ", name)
        return
    await RenderingServer.frame_post_draw
    var image := root.get_texture().get_image()
    if image == null or image.is_empty():
        check(false, "Empty viewport: " + name)
        return
    var path := ProjectSettings.globalize_path("res://.tmp/lowpoly/" + name + ".png")
    check(image.save_png(path) == OK, "Could not save " + path)
    captures.append(path)
    print("CAPTURE ", path)

func run() -> void:
    DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path("res://.tmp/lowpoly"))
    Engine.max_fps = 60
    root.size = Vector2i(1440, 900)
    scene = load("res://Levels/lowpoly.tscn").instantiate()
    root.add_child(scene)
    current_scene = scene
    var loading_started := Time.get_ticks_msec()
    while not scene.get("IsInitialized") and Time.get_ticks_msec() - loading_started < 30000:
        await process_frame
    if not scene.get("IsInitialized"):
        check(false, "3D scene failed to initialize within 30 seconds")
        quit(1)
        return
    await process_frame
    var cars := scene.get_children().filter(func(child): return child.name.begins_with("Car_"))
    check(cars.size() == 1, "Default practice must contain exactly one car")
    check(scene.get("SelectedCarIndex") == 0, "Follow camera must select the only car")
    check(not scene.get("IsPaused"), "Display should start unpaused")
    var stationary_car: Node3D = scene.get_node("Car_01")
    var stationary_position := stationary_car.global_position
    await create_timer(1.0).timeout
    check(is_zero_approx(float(scene.get("RaceSeconds"))), "No-controller presentation must not auto-step")
    check(stationary_position.distance_to(stationary_car.global_position) < 0.0001, "No-controller car must remain stationary")
    var controller_status := ""
    for label in scene.find_children("*", "Label", true, false):
        controller_status += label.text + "\n"
    check("NO CONTROLLER" in controller_status, "HUD must state NO CONTROLLER")
    press(KEY_SPACE)
    await create_timer(0.5).timeout
    check(scene.get("IsPaused"), "Pause must be reflected by the HUD state")
    var original_strategy := strategy_readout()
    check(not original_strategy.is_empty(), "Strategy readout is missing")
    for keys in [[KEY_E, KEY_Q], [KEY_D, KEY_A]]:
        press(keys[0])
        await create_timer(0.3).timeout
        check(strategy_readout() != original_strategy, "Strategy control must update the fitted car's dashboard")
        press(keys[1])
        await create_timer(0.3).timeout
        check(strategy_readout() == original_strategy, "Strategy control must restore its previous rung")
    var paused_started := Time.get_ticks_msec()
    var paused_frames := 0
    while Time.get_ticks_msec()-paused_started < 2000:
        await process_frame
        paused_frames += 1
    print("SMOKE paused frames=", paused_frames, " over ms=", Time.get_ticks_msec()-paused_started)
    var car: Node3D = scene.get_node("Car_01")
    var camera: Camera3D = scene.get_node("CameraRig/Lens")
    check(camera.projection == Camera3D.PROJECTION_PERSPECTIVE, "Chase must use perspective")
    check((camera.global_position - car.global_position).dot(car.global_basis.x) < -5, "Chase must sit behind the car")
    check(not camera.is_position_behind(car.global_position), "Chase must look toward the car")
    check(root.get_visible_rect().has_point(camera.unproject_position(car.global_position)), "Chase must frame the car")
    await capture("01-chase")
    for mode in [2, 3]:
        press(KEY_2 if mode == 2 else KEY_3)
        await create_timer(0.3).timeout
        check(scene.get("CameraMode") == mode, "Camera shortcut did not switch")
        check(camera.projection == Camera3D.PROJECTION_ORTHOGONAL, "Side and overview must use orthographic projection")
        if mode == 2:
            check(abs((camera.global_position - car.global_position).dot(car.global_basis.z)) > 20, "Side must view from beside the car")
            check(camera.size < 60, "Side must frame local track, not the whole circuit")
        else:
            check(camera.size > 1000, "Overview must frame the complete circuit")
        await capture("02-side" if mode == 2 else "03-overview")
    press(KEY_1)
    await process_frame
    press(KEY_RIGHT)
    await create_timer(0.3).timeout
    check(scene.get("SelectedCarIndex") == 0, "Single-car selection must remain valid")
    var distance_before := camera.global_position.distance_to(car.global_position)
    var wheel := InputEventMouseButton.new()
    wheel.button_index = MOUSE_BUTTON_WHEEL_UP
    wheel.pressed = true
    Input.parse_input_event(wheel)
    await create_timer(0.5).timeout
    check(camera.global_position.distance_to(car.global_position) < distance_before, "Wheel did not zoom chase camera")
    root.size = Vector2i(1024, 640)
    await create_timer(0.3).timeout
    await capture("04-compact")
    root.size = Vector2i(1440, 900)
    var frozen: float = scene.get("RaceSeconds")
    # Start a real external-input session while still paused. No controller is installed.
    scene.call("SetExternalInput", 0, 0.0, 2.0, 0.0)
    await create_timer(0.2).timeout
    check(is_equal_approx(frozen, float(scene.get("RaceSeconds"))), "Queued input must not bypass pause")
    var motion_start := car.global_position
    press(KEY_SPACE)
    var start := Time.get_ticks_msec()
    var frame_count := 0
    while Time.get_ticks_msec() - start < 6000:
        await process_frame
        frame_count += 1
    check(scene.get("RaceSeconds") > frozen, "Resume must advance simulation")
    check(car.global_position.distance_to(motion_start) > 1.0, "Explicit input must drive the vehicle")
    # A zero command is coasting: it must never freeze the clock or pose.
    scene.call("SetExternalInput", 0, 0.0, 0.0, 0.0)
    var coasting_time: float = scene.get("RaceSeconds")
    var coasting_position := car.global_position
    await create_timer(0.5).timeout
    check(scene.get("RaceSeconds") > coasting_time + 0.1, "Zero command must keep physics running")
    check(car.global_position.distance_to(coasting_position) > 0.01, "Zero command must not freeze a moving car")
    await capture("05-running")
    print("SMOKE rendered frames=", frame_count, " over ms=", Time.get_ticks_msec()-start, " simulation_seconds=", scene.get("RaceSeconds"))
    press(KEY_SPACE)
    await create_timer(1.0).timeout
    var stopped: float = scene.get("RaceSeconds")
    await create_timer(0.4).timeout
    check(is_equal_approx(stopped,scene.get("RaceSeconds")), "Pause must settle an in-flight worker and stop further steps")
    var report := {"passed": failures.is_empty(), "headless": DisplayServer.get_name() == "headless", "errors": failures, "captures": captures, "race_seconds": scene.get("RaceSeconds")}
    var output := FileAccess.open("res://.tmp/lowpoly/smoke-report.json", FileAccess.WRITE)
    output.store_string(JSON.stringify(report, "  "))
    output.close()
    finished = true
    print("SMOKE ", "PASS" if failures.is_empty() else "FAIL", " errors=", failures)
    quit(0 if failures.is_empty() else 1)
