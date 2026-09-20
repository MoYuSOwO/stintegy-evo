// The training adapter is compiled into the game as it stands, so that the
// observation the policy is fed in a race is built by the same source that
// built it in the bake. That project has implicit usings on and this one
// does not, so the namespaces it relies on are declared here rather than by
// editing a file whose whole value is being unedited. System.IO is
// deliberately absent: it would make FileAccess ambiguous with Godot's.
global using System;
global using System.Collections.Generic;
