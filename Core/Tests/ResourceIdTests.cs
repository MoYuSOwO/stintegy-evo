using StintegyEVO.Core.Content;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Everything a mod registers is named by an id: its mod's namespace and
/// its own name, a bare name being the game's own.
/// </summary>
public sealed class ResourceIdTests
{
    [Theory]
    [InlineData("official:formula_a_mixed", "official", "formula_a_mixed")]
    [InlineData("formula_a_mixed", "official", "formula_a_mixed")]
    [InlineData("someone:their_car_2", "someone", "their_car_2")]
    public void IdsReadWithOrWithoutTheirNamespace(string text, string space, string name)
    {
        ResourceId id = ResourceId.Parse(text);
        Assert.Equal(space, id.Namespace);
        Assert.Equal(name, id.Name);
        Assert.Equal($"{space}:{name}", id.ToString());
    }

    [Theory]
    [InlineData("Official:Car")]
    [InlineData("official:")]
    [InlineData(":car")]
    [InlineData("official:my-car")]
    public void AnIdIsLowerCaseLettersDigitsAndUnderscores(string text)
    {
        Assert.False(ResourceId.TryParse(text, out _));
    }
}
