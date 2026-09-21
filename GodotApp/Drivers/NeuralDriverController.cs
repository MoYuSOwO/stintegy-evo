using System;
using System.Collections.Generic;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;

namespace StintegyEVO.GodotApp.Drivers;

/// <summary>
/// The baked policy, driving a car in the game.
///
/// This is the same seat the training host puts the policy in, with the
/// clock moved inside. In training the environment owns the decision
/// period and calls observe, then commit, around each step; in a race
/// nobody is outside the simulation to do that, so this counts the
/// simulation's own driving substeps and decides on the same fifteen-hertz
/// beat, holding the command in between. What the network sees and what
/// the car is asked to do are built by
/// <see cref="DirectDriveController"/> itself — the training adapter, used
/// as it stands — so the observation cannot drift from the one the policy
/// was baked against by anybody editing this file.
///
/// The physics is live. A spin here is a real spin, a save is a real save,
/// and nothing replays: the car is where the simulation put it. Being
/// deterministic, the same start gives the same lap every run, which is a
/// property of a noise-free deployment rather than a recording.
/// </summary>
public sealed class NeuralDriverController : IDriverController, IDisposable
{
    /// <summary>
    /// The decision rate the policy was trained at. A deployed driver
    /// thinking at another rate is a different driver: the action is held
    /// for exactly this long in training, and the reaction latency that
    /// creates is part of what the policy learned to drive around.
    /// </summary>
    public const float DecisionHz = DirectDriveController.DefaultDecisionHz;

    private readonly DirectDriveController _seat;
    private readonly InferenceSession _session;
    private readonly DenseTensor<float> _input =
        new(new[] { 1, DirectDriveObservation.ObservationSize });
    private readonly List<NamedOnnxValue> _inputs;
    private readonly float[] _action = new float[DirectDriveObservation.ActionSize];
    private readonly string _inputName;
    private readonly string _outputName;
    private readonly float _period = 1f / DecisionHz;
    private readonly float _raceMeters;
    private readonly int _powerRung;

    private float _sinceDecision;
    private bool _started;
    private EnergyBudget.Anchor _anchor;
    private float _progressAtStart;
    private float _distanceOrigin;

    /// <summary>
    /// Whether this driver's first action moves the steering command
    /// rather than being it. Read from the network's card, never guessed:
    /// driving an incremental policy as an absolute one, or the other way
    /// round, produces a car that looks broken rather than one that looks
    /// wrong.
    /// </summary>
    public bool DeltaActions { get; }

    /// <summary>How many decisions this driver has taken.</summary>
    public long Decisions { get; private set; }

    /// <summary>The last command asked for, in the policy's own units.</summary>
    public float LastCurvatureNorm => _action[0];
    public float LastAccelerationNorm => _action[1];

    /// <summary>The observation the last decision was taken on.</summary>
    public ReadOnlySpan<float> LastObservation => _seat.LastObservation;

    /// <summary>Milliseconds the last inference took.</summary>
    public double LastInferenceMs { get; private set; }

    public NeuralDriverController(
        string modelPath,
        CarConfig config,
        TireConfig tires,
        CarStrategy strategy,
        float raceKilometres = EnergyBudget.DefaultRaceKilometres
    )
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(modelPath);
        // What the first action means is a property of the bake, not of
        // the network file, so it travels beside it: the exporter writes
        // the checkpoint's own record of it into a card next to the
        // network. A missing or unreadable card means the contract every
        // generation up to the fourth was baked on.
        DeltaActions = ReadsAsDelta(modelPath);
        _seat = new DirectDriveController(
            config, tires, DeltaActions, DecisionHz
        );
        SessionOptions options = new()
        {
            // One car, one row, on the frame's own thread: the work is a
            // few hundred microseconds and a thread pool per driver would
            // cost more than it saves.
            IntraOpNumThreads = 1,
            InterOpNumThreads = 1,
        };
        _session = new InferenceSession(modelPath, options);
        _inputName = FirstName(_session.InputMetadata, "observation");
        _outputName = FirstName(_session.OutputMetadata, "action");
        _inputs = [NamedOnnxValue.CreateFromTensor(_inputName, _input)];
        _raceMeters = raceKilometres * 1000f;
        _powerRung = strategy.PowerRung;
    }

    public void Initialize(in DriverContext context)
    {
        _sinceDecision = 0f;
        _started = false;
        Decisions = 0;
    }

    /// <summary>
    /// The held command, and a fresh decision whenever the period is up.
    ///
    /// The clock is read before the substep is counted, so decisions land
    /// on the beat — at zero, at one period, at two — rather than a substep
    /// early. The remainder is carried rather than cleared, so a decision
    /// rate that does not divide the substep length still averages out to
    /// fifteen a second instead of drifting slower. The first driving
    /// substep decides, so the car is never driven by a default command it
    /// never chose; that first observation is the only one taken before any
    /// physics has run, and its tyre loads are therefore zero for one
    /// substep.
    /// </summary>
    public DriverInput GetControl(in DriverContext context, float dt)
    {
        if (!_started || _sinceDecision + 1e-6f >= _period)
        {
            _sinceDecision = _started ? _sinceDecision - _period : 0f;
            Decide(in context);
            _started = true;
        }
        _sinceDecision += dt;
        return _seat.GetControl(in context, dt);
    }

    private void Decide(in DriverContext context)
    {
        RaceCarSnapshot car = context.Car;
        AnchorIfNeeded(in car);
        _seat.BudgetDeviation = BudgetDeviation(in car);
        _seat.Observe(in context);

        ReadOnlySpan<float> observation = _seat.LastObservation;
        for (int i = 0; i < observation.Length; i++)
            _input.Buffer.Span[i] = observation[i];

        long started = System.Diagnostics.Stopwatch.GetTimestamp();
        using IDisposableReadOnlyCollection<DisposableNamedOnnxValue> results =
            _session.Run(_inputs, [_outputName]);
        LastInferenceMs =
            System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds;

        Tensor<float> action = results[0].AsTensor<float>();
        for (int i = 0; i < _action.Length; i++)
        {
            float value = i < action.Length ? action.GetValue(i) : 0f;
            _action[i] = float.IsFinite(value) ? Math.Clamp(value, -1f, 1f) : 0f;
        }
        _seat.CommitAction(_action);
        Decisions++;
    }

    /// <summary>
    /// The budget line, anchored the way an evaluation anchors it: where
    /// the car is on the Normal race path for the charge it started with,
    /// and the charge it started with. The policy reads one channel of
    /// this, and an unanchored line would feed it a number training never
    /// produced.
    /// </summary>
    private void AnchorIfNeeded(in RaceCarSnapshot car)
    {
        if (_started)
            return;
        float charge = Charge(in car);
        _progressAtStart = EnergyBudget.ProgressOnNormalRace(charge);
        _distanceOrigin = car.TotalDistanceMeters;
        _anchor = new EnergyBudget.Anchor(_progressAtStart, charge);
    }

    private float BudgetDeviation(in RaceCarSnapshot car)
    {
        float progress = _progressAtStart +
            (car.TotalDistanceMeters - _distanceOrigin) / _raceMeters;
        return EnergyBudget.Deviation(
            _anchor, progress, Charge(in car), _powerRung
        );
    }

    private static float Charge(in RaceCarSnapshot car) =>
        car.Resources.IsDefaultOrEmpty ? 0f : car.Resources[0].Fraction;

    /// <summary>
    /// The card beside the network: same name, .json. Read for one field,
    /// with a plain string search rather than a parser, because the file
    /// is written by our own exporter and a missing card has to be an
    /// absolute policy rather than an exception.
    /// </summary>
    private static bool ReadsAsDelta(string modelPath)
    {
        // Qualified: System.IO is deliberately absent from this project's
        // global usings, because it makes FileAccess ambiguous with Godot's.
        string card = System.IO.Path.ChangeExtension(modelPath, ".json");
        if (!System.IO.File.Exists(card))
            return false;
        try
        {
            string text = System.IO.File.ReadAllText(card);
            int at = text.IndexOf("\"action_semantics\"", StringComparison.Ordinal);
            if (at < 0)
                return false;
            int colon = text.IndexOf(':', at);
            int quote = colon < 0 ? -1 : text.IndexOf('"', colon + 1);
            int end = quote < 0 ? -1 : text.IndexOf('"', quote + 1);
            if (end < 0)
                return false;
            return text[(quote + 1)..end].Trim() == "delta";
        }
        catch (Exception error) when (
            error is System.IO.IOException or UnauthorizedAccessException
        )
        {
            return false;
        }
    }

    private static string FirstName(
        IReadOnlyDictionary<string, NodeMetadata> metadata, string preferred
    )
    {
        if (metadata.ContainsKey(preferred))
            return preferred;
        foreach (string name in metadata.Keys)
            return name;
        throw new InvalidOperationException(
            $"The model has no {preferred} to bind to."
        );
    }

    public void Dispose() => _session.Dispose();
}
