using System;
using System.IO;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Saves;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// A part keeps what it learns in a slot it names by whose and about what,
/// never by a path; the slot is the same to it whether the save is on disk
/// or thrown away with the session.
/// </summary>
public sealed class SaveStoreTests : IDisposable
{
    private static readonly ResourceId Lanes = new("someone", "lanes");
    private static readonly ResourceId Car = new("official", "formula_a_mixed");
    private static readonly ResourceId Silverstone = new("official", "silverstone");

    private readonly string _root = Path.Combine(Path.GetTempPath(), "stintegy-saves-" + Guid.NewGuid().ToString("N"));

    public void Dispose()
    {
        if (Directory.Exists(_root))
            Directory.Delete(_root, recursive: true);
    }

    /// <summary>
    /// Each owner's data about each subject is its own: another person,
    /// driver, car or circuit is another slot, and leaving a part of the
    /// subject out is a slot of its own too.
    /// </summary>
    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ASlotIsItsOwnersAboutItsSubject(bool onDisk)
    {
        ISaveStore save = onDisk ? new SaveFolder(_root) : new ThrowawaySave();
        var key = new SaveKey(Lanes, "ana", Car, Silverstone);
        save.Slot(key).Write("lines.csv", "a");

        Assert.Equal("a", save.Slot(key).Read("lines.csv"));
        Assert.Null(save.Slot(key).Read("other.csv"));
        Assert.Null(save.Slot(new SaveKey(Lanes, "ben", Car, Silverstone)).Read("lines.csv"));
        Assert.Null(save.Slot(new SaveKey(new ResourceId("official", "lanes"), "ana", Car, Silverstone)).Read("lines.csv"));
        Assert.Null(save.Slot(new SaveKey(Lanes, "ana", new ResourceId("someone", "formula_a_mixed"), Silverstone)).Read("lines.csv"));
        Assert.Null(save.Slot(new SaveKey(Lanes, "ana", Car, new ResourceId("official", "spa"))).Read("lines.csv"));
        Assert.Null(save.Slot(new SaveKey(Lanes, "ana", Car)).Read("lines.csv"));

        save.Slot(key).Write("lines.csv", "b");
        Assert.Equal("b", save.Slot(key).Read("lines.csv"));
    }

    /// <summary>
    /// On disk every id is split into folders, so no colon reaches a path,
    /// and a write leaves nothing half done beside the file.
    /// </summary>
    [Fact]
    public void OnDiskEveryIdIsSplitIntoFolders()
    {
        var save = new SaveFolder(_root);
        save.Slot(new SaveKey(Lanes, "ana", Car, Silverstone)).Write("lines.csv", "a");
        save.Slot(new SaveKey(Lanes)).Write("settings.json", "{}");

        string[] files = Directory.GetFiles(_root, "*", SearchOption.AllDirectories);
        Assert.Equal(2, files.Length);
        foreach (string file in files)
            Assert.DoesNotContain(':', Path.GetRelativePath(_root, file));
    }

    /// <summary>A slot's files are inside it, and a person is named only as a folder can be named everywhere.</summary>
    [Fact]
    public void WhatIsNotInsideASlotIsRefused()
    {
        ISaveSlot slot = new ThrowawaySave().Slot(new SaveKey(Lanes, "ana"));
        Assert.Throws<ArgumentException>(() => slot.Read("../elsewhere"));
        Assert.Throws<ArgumentException>(() => slot.Write("/tmp/elsewhere", ""));
        Assert.Throws<ArgumentException>(() => new SaveKey(Lanes, "Ana"));
        Assert.Throws<ArgumentException>(() => new SaveKey(Lanes, "../x"));
    }
}
