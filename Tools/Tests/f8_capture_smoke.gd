extends SceneTree
# Headless integration: exercise real viewport input, not a recorder API.
const DIR = "/tmp/stintegy-telemetry"
func clips():
    return Array(DirAccess.get_files_at(DIR)) if DirAccess.dir_exists_absolute(DIR) else []
func press(echo := false):
    var event := InputEventKey.new()
    event.keycode = KEY_F8
    event.pressed = true
    event.echo = echo
    root.push_input(event)
func _initialize():
    run.call_deferred()
func run():
    var before = clips()
    change_scene_to_file("res://Tools/Demo/lane_driver_demo.tscn")
    await scene_changed
    for i in 2:
        press()
        await create_timer(0.25).timeout
        press(true)
        await create_timer(0.50).timeout
        press()
        await create_timer(0.25).timeout
    var added = clips().filter(func(f): return not before.has(f))
    if added.size() != 2:
        printerr("Expected two F8 clips, got ", added.size()); quit(1); return
    for name in added:
        var f = FileAccess.open(DIR + "/" + name, FileAccess.READ)
        var header = Array(f.get_csv_line()); var frames = {}; var last = -1
        for field in ["physics_tick", "trajectory_version", "target_speed_mps", "actual_wheel_angle_rad", "brake_request_mps2"]:
            if not header.has(field): printerr("Missing ", field); quit(1); return
        while f.get_position() < f.get_length():
            var row = f.get_csv_line(); var tick = int(row[header.find("physics_tick")])
            if row.size() != header.size(): printerr("Bad aligned row"); quit(1); return
            if not frames.has(tick):
                if last >= 0 and tick != last + 1: printerr("Skipped tick"); quit(1); return
                frames[tick] = []; last = tick
            frames[tick].append(row[header.find("car_id")])
        if frames.size() < 2: printerr("No complete frames"); quit(1); return
        var expected_cars = -1
        for cars in frames.values():
            var unique = {}
            for car in cars: unique[car] = true
            if expected_cars < 0: expected_cars = cars.size()
            if cars.size() < 1 or cars.size() != expected_cars or unique.size() != cars.size(): printerr("Incomplete grid"); quit(1); return
        print("F8 verified: ", name, " ", frames.size(), " ticks x ", expected_cars, " cars")
    quit(0)
