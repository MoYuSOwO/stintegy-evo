using System;
using System.Collections.Generic;
using System.Linq;
using Godot;
using StintegyEVO.GodotApp.LowPoly;

namespace StintegyEVO.GodotApp.Scenery;

/// <summary>
/// Placing scenery by pointing at the ground.
///
/// The alternative was the Godot editor: place nodes in a scene, run a
/// script that converts them to track coordinates, save the plan. That
/// path is written down in the design note and it is not the one taken,
/// for a reason worth stating: the circuit does not exist in the editor.
/// It is built at load from TrackData — road, kerbs, barriers and terrain,
/// all procedural — so an author placing props there is placing them
/// against an empty grid and finding out where they landed by running the
/// game. Making the circuit build in the editor is a bigger change than
/// this whole feature.
///
/// The viewer already draws the circuit. So the editor is the viewer: fly
/// the camera where you want the thing, point at the ground, click. What
/// is written out is a plan file in the same format a person would type,
/// in track coordinates, one prop a line.
///
/// Nothing placed here is known to the physics. A tree is scenery; the
/// barrier is the boundary.
/// </summary>
public sealed partial class SceneryEditMode : Node3D
{
    private readonly List<SceneryPlacement> _placed = [];
    private readonly List<Node3D> _shown = [];
    private string[] _library = [];
    private int _selected;
    private TrackSurfaceGeometry _surface = null!;
    private SceneryLoader _loader = null!;
    private Label _hud = null!;
    private string _track = "silverstone";
    private string _planPath = string.Empty;

    /// <summary>Whether placement is taking clicks.</summary>
    public bool Active { get; private set; }

    public void Initialize(
        TrackSurfaceGeometry surface,
        SceneryLoader loader,
        string track,
        string planPath
    )
    {
        _surface = surface;
        _loader = loader;
        _track = track;
        _planPath = planPath;
        _library = ListLibrary();
        if (Godot.FileAccess.FileExists(planPath))
        {
            using var file = Godot.FileAccess.Open(planPath, Godot.FileAccess.ModeFlags.Read);
            _placed.AddRange(SceneryPlan.Parse(file.GetAsText()).Props);
        }

        CanvasLayer layer = new() { Name = "SceneryHud" };
        _hud = new Label
        {
            Position = new Vector2(44, 640),
            Modulate = new Color(0.16f, 0.16f, 0.16f)
        };
        _hud.AddThemeFontSizeOverride("font_size", 15);
        layer.AddChild(_hud);
        AddChild(layer);
        Refresh();
    }

    public void Toggle()
    {
        Active = !Active;
        Refresh();
    }

    public override void _UnhandledInput(InputEvent input)
    {
        if (input is InputEventKey key && key.Pressed && !key.Echo &&
            key.Keycode == Key.F2)
        {
            Toggle();
            GetViewport().SetInputAsHandled();
            return;
        }
        if (!Active || _library.Length == 0)
            return;

        if (input is InputEventKey edit && edit.Pressed && !edit.Echo)
        {
            switch (edit.Keycode)
            {
                case Key.Bracketleft:
                    _selected = (_selected - 1 + _library.Length) % _library.Length;
                    break;
                case Key.Bracketright:
                    _selected = (_selected + 1) % _library.Length;
                    break;
                case Key.Z:
                    Undo();
                    break;
                case Key.S when edit.CtrlPressed || edit.MetaPressed:
                    Save();
                    break;
                default:
                    return;
            }
            Refresh();
            GetViewport().SetInputAsHandled();
            return;
        }

        if (input is InputEventMouseButton click && click.Pressed &&
            click.ButtonIndex == MouseButton.Left)
        {
            Place(click.Position);
            GetViewport().SetInputAsHandled();
        }
    }

    /// <summary>
    /// Where the cursor is pointing on the ground, as a station and an
    /// offset. The ray is walked until it goes under the terrain and then
    /// halved a few times, which costs nothing and copes with the meadow's
    /// rolling and the circuit's own climb alike.
    /// </summary>
    private void Place(Vector2 cursor)
    {
        Camera3D? camera = GetViewport().GetCamera3D();
        if (camera is null)
            return;
        Vector3 from = camera.ProjectRayOrigin(cursor);
        Vector3 direction = camera.ProjectRayNormal(cursor);
        if (!Ground(from, direction, out Vector3 hit))
            return;

        (float s, float d) = _surface.NearestStation(
            new System.Numerics.Vector2(hit.X, hit.Z)
        );
        SceneryPlacement prop = new(_library[_selected], s, d);
        _placed.Add(prop);
        Node3D node = new() { Name = $"preview_{_placed.Count}" };
        AddChild(node);
        _shown.Add(node);
        _loader.Show(prop, _surface, node);
        Refresh();
    }

    private bool Ground(Vector3 from, Vector3 direction, out Vector3 hit)
    {
        hit = from;
        const float step = 2f;
        for (float travelled = 0f; travelled < 2000f; travelled += step)
        {
            Vector3 point = from + direction * travelled;
            if (point.Y > _surface.MeadowHeight(point.X, point.Z))
                continue;
            // Between the last step and this one: halve a few times, which
            // is plenty for ground a person is pointing at.
            Vector3 above = from + direction * (travelled - step);
            for (int i = 0; i < 12; i++)
            {
                Vector3 middle = (above + point) * 0.5f;
                if (middle.Y > _surface.MeadowHeight(middle.X, middle.Z))
                    above = middle;
                else
                    point = middle;
            }
            hit = point;
            return true;
        }
        return false;
    }

    private void Undo()
    {
        if (_placed.Count == 0)
            return;
        _placed.RemoveAt(_placed.Count - 1);
        if (_shown.Count > 0)
        {
            _shown[^1].QueueFree();
            _shown.RemoveAt(_shown.Count - 1);
        }
    }

    private void Save()
    {
        SceneryPlan plan = new() { Track = _track, Props = _placed.ToArray() };
        string path = ProjectSettings.GlobalizePath(_planPath);
        try
        {
            System.IO.Directory.CreateDirectory(
                System.IO.Path.GetDirectoryName(path)!
            );
            System.IO.File.WriteAllText(path, plan.ToJson());
            GD.Print($"SCENERY saved {_placed.Count} props to {path}");
        }
        catch (Exception error)
        {
            GD.PushError($"scenery: could not write {path} -- {error.Message}");
        }
    }

    private static string[] ListLibrary()
    {
        using DirAccess? directory = DirAccess.Open(SceneryLoader.LibraryPath);
        if (directory is null)
            return [];
        return directory.GetFiles()
            .Where(name => name.EndsWith(".tscn", StringComparison.Ordinal) ||
                           name.EndsWith(".glb", StringComparison.Ordinal) ||
                           name.EndsWith(".gltf", StringComparison.Ordinal) ||
                           name.EndsWith(".scn", StringComparison.Ordinal))
            .Select(name => name[..name.LastIndexOf('.')])
            .Distinct()
            .OrderBy(name => name, StringComparer.Ordinal)
            .ToArray();
    }

    private void Refresh()
    {
        if (!Active)
        {
            _hud.Text = "F2  place scenery";
            return;
        }
        string prop = _library.Length > 0 ? _library[_selected] : "(library empty)";
        _hud.Text =
            $"SCENERY  [{prop}]   {_placed.Count} placed\n" +
            "click place    [ ] prop    Z undo    ctrl+S save    F2 done";
    }
}
